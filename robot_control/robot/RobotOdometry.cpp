#include "RobotOdometry.h"

RobotOdometry::RobotOdometry() {
	map_size= Size(600, 600);
	background = Mat::zeros(map_size.width, map_size.height, CV_8UC3);
	robot_shape = Mat::zeros(map_size.width, map_size.height, CV_8UC3);
	map = Mat::zeros(map_size.width, map_size.height, CV_8UC3);
	position=Point2d(0, 0);
	enc_left = 0;
	enc_right = 0;
	enc_left_old = 0;
	enc_right_old = 0;
	enc_diff_left = 0;
	enc_diff_right = 0;
	left = 0;
	right = 0;
	enc_l_dir = 1;
	enc_r_dir = 1;

	azimuth = 0;
	angleDeg = 5.5;
	angle_step = angleDeg*3.14159265 / 180;
	move_step = 0.5;
	view_dist = 60;
	view_angleDeg = 30;
	view_angle = view_angleDeg*3.14159265 / 180;
	view_res = 3;
	center_x = 300;
	center_y = 350;

	posx_base = 52.4296548;
	posy_base = 13.5382382;
};

RobotOdometry::~RobotOdometry() {};

void RobotOdometry::decodeEncoders() {
	cout << "enc_left: " << enc_left << " enc_right: " << enc_right << endl;
	enc_diff_left = (enc_l_dir < 0) ? -enc_left + enc_left_old : enc_left - enc_left_old;
	enc_diff_right = (enc_r_dir < 0) ? -enc_right + enc_right_old : enc_right - enc_right_old;
	if (enc_diff_left < 40 && enc_diff_right < 40) {
		left = enc_diff_left;
		right = enc_diff_right;
		enc_left_old = enc_left;
		enc_right_old = enc_right;
	}
}

void RobotOdometry::updateCoordinates() {
	azimuth += (left - right)*angle_step;
	position.x += ((left + right)*move_step)*sin(azimuth);
	position.y += -((left + right)*move_step)*cos(azimuth);
	cout << "left: " << left << " right: " << right << " azimuth: " << azimuth <<"position: "<<position.x<<" "<<position.y << endl;

	posx = posx_base + position.y*0.00000005;
	posy = posy_base - position.x*0.00000005;
}

void RobotOdometry::drawRobot(Mat& image, Point centerPoint, Size rectangleSize, double rotationDegrees) {
	image = Mat::zeros(image.cols, image.rows, image.type());
	Scalar color = cv::Scalar(100, 255, 0);
	// Create the rotated rectangle
	RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
	// We take the edges that OpenCV calculated for us
	Point2f vertices2f[4];
	rotatedRectangle.points(vertices2f);
	// Convert them so we can use them in a fillConvexPoly
	Point vertices[4];
	for (int i = 0; i < 4; ++i) {
		vertices[i] = vertices2f[i];
	}
	// Now we can fill the rotated rectangle with our specified color
	cv::fillConvexPoly(image,
		vertices,
		4,
		color);
}

void RobotOdometry::drawCurrentArea(Mat& background, Point center, double azimuth) {
	Scalar color = Scalar(70, 70, 100);

	Point vertices[4];
	for (int i = -1; i < 2; i++) {
		vertices[i + 1].x = center.x + view_dist*sin(azimuth) + cos(azimuth)*sin(view_angle*i)*view_dist;
		vertices[i + 1].y = center.y - view_dist*cos(azimuth) + sin(azimuth)*sin(view_angle*i)*view_dist;
	}
	vertices[3] = center;

	fillConvexPoly(background, vertices, 4, color);
}

void RobotOdometry::updateMap()
{
	//map = Mat::zeros(map.cols, map.rows, map.type());
	int x = position.x + center_x;
	int y = position.y + center_y;
	circle(map, Point(x, y), 1, CV_RGB(0, 0, 255), 2);
	drawRobot(robot_shape, Point(x, y), Size(10, 15), azimuth * 180 / 3.14);
	drawCurrentArea(background, Point(x, y), azimuth);
}

void RobotOdometry::markTarget() {
	int x1 = 1;
	int y1 = -40;
	int x2 = x1 / cos(atan(y1 / x1))*cos(atan(y1 / x1) + azimuth) + position.x + center_x;
	int y2 = x1 / cos(atan(y1 / x1))*sin(atan(y1 / x1) + azimuth) + position.y + center_y;
	putText(map, "object", Point(x2 + 3, y2), CV_FONT_HERSHEY_COMPLEX, 0.25, Scalar(30, 255, 30), 0.5, LINE_4, 0);
	circle(map, Point(x2, y2), 2, CV_RGB(30, 255, 30), 0.5);
}

void RobotOdometry::map3d(Mat &map, Mat image3d) {
	int j = 120;
	//map = Mat::zeros(600, 1280, CV_8UC3);
	//Rect robot_rect(center_x - 10, center_y - 10, 20, 30);
	//rectangle(map, robot_rect, Scalar(30, 255, 60), 2);
	for (int j = 50; j < 51; j++) {
		for (int i = 20; i < image3d.cols - 20; i++) {

			/*int x = int(5 * image3d.at<Vec3f>(j, i)[0]) + center_x);
			int y = int(-5 * image3d.at<Vec3f>(j, i)[2]) + center_y;*/

			int x1 = ((3 * image3d.at<Vec3f>(j, i)[0]));
			int y1 = (-3 * image3d.at<Vec3f>(j, i)[2]);
			//cout << "zaraz dodam " <<y1<<" "<<x1<< endl;
			if (x1 != 0.0&& cos(atan(y1 / x1)) != 0.0) {
				int x2 = x1 / cos(atan(y1 / x1))*cos(atan(y1 / x1) + azimuth) + position.x + center_x;
				int y2 = x1 / cos(atan(y1 / x1))*sin(atan(y1 / x1) + azimuth) + position.y + center_y;
				//int x2 = x1 + position.x + center_x;
				//int y2 = y1 + position.y + center_y;
				circle(map, Point(x2, y2), 0.5, CV_RGB(255, 0, 0), 0.5);
			}
			//cout << "dodalem" << endl;
		}
	}
}