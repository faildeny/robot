#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <thread>
extern "C" {
#include "gopigo.h"
}

#include "StereoCamera.h"
#include "Streamer.h"
#include "RobotControl.h"

using namespace cv;
using namespace std;


String object_cascade_name = "cascade.xml";
CascadeClassifier object_cascade;
RNG rng(12345);

int speed=60;

//Position Coordinates
static double posx_base = 52.4296548;
static double posy_base = 13.5382382;

double posx;
double posy;

//Parser
void parseArguments(int argc, char** argv) {
	for (int i = 0; i < argc; i++) {
		if (string(argv[i]) == "-v" && i + 1<argc) {
			speed = atoi(argv[i + 1]);
			cout << "set speed: " << speed << endl;
		}
	}
}
bool detectAndDisplay(Mat frame, Rect &target)
{
	std::vector<Rect> faces;
	Rect none(frame.cols*0.5, frame.rows*0.5, 10, 10);
	Mat frame_gray;

	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	object_cascade.detectMultiScale(frame_gray, faces, 1.05, 1, 0 | CV_HAAR_SCALE_IMAGE, Size(48, 30));
	for (int i = 0; i < faces.size(); i++)
	{
		rectangle(frame, faces[i], Scalar(0, 255, 200), 2, 8);
		target = faces[0];
	}
	if (faces.size() == 0) {
		target = none;
		return 0;
	}
	else {
		imshow("oknno", frame);
		return 1;
	}
}


// Odometry module
Size map_size(600, 600);
Mat background= Mat::zeros(map_size.width, map_size.height, CV_8UC3);
Mat robot_shape = Mat::zeros(map_size.width, map_size.height, CV_8UC3);
Mat map = Mat::zeros(map_size.width, map_size.height, CV_8UC3);
Point2d position(0, 0);
int enc_left = 0;
int enc_right = 0;
int enc_left_old = 0;
int enc_right_old = 0;
int enc_diff_left = 0;
int enc_diff_right = 0;
int enc_l_dir = 1;
int enc_r_dir = 1;

double azimuth = 0;
double angleDeg = 4.5;
double angle_step = angleDeg*3.14159265 / 180;
double move_step = 0.5;
int max_step=8;
int view_dist=60;
int view_angleDeg = 20;
double view_angle=view_angleDeg*3.14159265 / 180;
int view_res = 3;


void decodeEncoders() {
	cout << "enc_left: " << enc_left << " enc_right: " << enc_right << endl;
	enc_diff_left = (enc_l_dir < 0) ? -enc_left + enc_left_old : enc_left - enc_left_old;
	enc_diff_right = (enc_r_dir < 0) ? -enc_right + enc_right_old : enc_right - enc_right_old;
	enc_left_old = enc_left;
	enc_right_old = enc_right;
}

void updateCoordinates(int left,int right) {

	azimuth += (left-right)*angle_step;
	position.x += (left+right*move_step)*sin(azimuth);
	position.y += -(left+right*move_step)*cos(azimuth);
	cout << "left: " << left << " right: " << right << " azimuth: " << azimuth << endl;

	posx = posx_base - position.y*0.00000005;
	posy = posy_base + position.x*0.00000005;
}

void drawRobot(Mat& image, Point centerPoint, Size rectangleSize, double rotationDegrees) {
	image = Mat::zeros(image.cols,image.rows,image.type());
	Scalar color = cv::Scalar(100,255,0); 
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

void drawCurrentArea(Mat& background, Point center, double azimuth) {
		Scalar color = Scalar(70, 70, 100);
		
		Point vertices[4];
		for (int i = -1; i < 2; i++) {
			vertices[i+1].x = center.x + view_dist*sin(azimuth) + cos(azimuth)*sin(view_angle*i)*view_dist;
			vertices[i+1].y = center.y - view_dist*cos(azimuth) + sin(azimuth)*sin(view_angle*i)*view_dist;
		}
		vertices[3] = center;

		fillConvexPoly(background, vertices, 4, color);
}

void updateMap(Point2d position)
{
	int x = position.x + 300;
	int y = position.y + 350;
	circle(map, Point(x, y), 1, CV_RGB(0, 0, 255), 2);
	drawRobot(robot_shape, Point(x, y), Size(10, 15), azimuth * 180 / 3.14);
	drawCurrentArea(background, Point(x, y), azimuth);
}

Mat image3d;
Mat scan_line1, scan_line2;

void map3d(Mat &map, Mat image3d) {
	int center_x = 300;
	int center_y = 350;
	int j = 20;
	//map = Mat::zeros(600, 1280, CV_8UC3);
	//Rect robot_rect(center_x - 10, center_y - 10, 20, 30);
	//rectangle(map, robot_rect, Scalar(30, 255, 60), 2);
	for (int j = 20; j < 21; j++) {
		for (int i = 0; i < image3d.cols; i++) {

			/*int x = int(5 * image3d.at<Vec3f>(j, i)[0]) + center_x);
			int y = int(-5 * image3d.at<Vec3f>(j, i)[2]) + center_y;*/

			int x1 = ((-5 * image3d.at<Vec3f>(j, i)[0]));
			int y1 = int(-5 * image3d.at<Vec3f>(j, i)[2]);

			//int x2 = x1/cos(atan(y1/x1))*cos(atan(y1/x1)+azimuth-0.5)+position.x+center_x;
			//int y2 = y1 / sin(atan(y1 / x1))*sin(atan(y1 / x1) + azimuth-0.5) +position.y+center_y;
			int x2 = x1 +position.x+ center_x;
			int y2 = y1 + position.y + center_y;
			circle(map, Point(x2, y2), 1, CV_RGB(255, 0, 0), 2);
		}
	}
}

//End of odometry module

//Depth map processing

double farthest_dist, nearest_dist;
ostringstream ss;
Size disp_size;
Rect area_rect;
Range area_h;
Range area_w;
double centdistance;
String text;

int sum_l, sum_r;
int turn;
int border;
Range dir_area_l;
Range dir_area_r;
Range dir_area_h;
Scalar sum_l_scalar;
Scalar sum_r_scalar;

Rect left_area;
Rect right_area;

double distCentralArea(Mat disp, StereoCamera stereo) {
	
	disp_size = disp.size();

	area_rect=Rect(disp_size.width / 2 - disp_size.width / 4, disp_size.height / 2 - disp_size.height / 4, disp_size.width / 2, disp_size.height / 2);

	area_h=Range(disp_size.height / 2 - disp_size.height / 4, disp_size.height / 2 + disp_size.height / 4);
	area_w=Range(disp_size.width / 2 - disp_size.width / 4, disp_size.width / 2 + disp_size.width / 4);
	disp.convertTo(disp, CV_32FC1);
	minMaxLoc(disp(area_h, area_w), &farthest_dist, &nearest_dist);
	//float d = disp.at<float>(punkt);
	centdistance = 0.2 * 0.001 / stereo.Q.at<double>(3, 2)*stereo.Q.at<double>(2, 3) / nearest_dist*16.f;
	nearest_dist = centdistance;
	ss << nearest_dist;
	String text = ss.str();

	return centdistance;

}

double avoidDirection(Mat disp) {
	disp_size = disp.size();
	border = 50;
	dir_area_l=Range (border, disp_size.width*0.5);
	dir_area_r=Range (disp_size.width*0.5, disp_size.width - border);
	dir_area_h=Range (disp_size.height*0.3, disp_size.height*0.9);
	sum_l_scalar = sum(disp(dir_area_h, dir_area_l));
	sum_l = sum_l_scalar[0] / countNonZero(disp(dir_area_h, dir_area_l));
	sum_r_scalar = sum(disp(dir_area_h, dir_area_r));
	sum_r = sum_r_scalar[0] / countNonZero(disp(dir_area_h, dir_area_r));
	turn = sum_l - sum_r;

	left_area=Rect (border, disp_size.height*0.3, disp_size.width*0.5 - border, disp_size.height*0.6);
	right_area=Rect (disp_size.width*0.5, disp_size.height*0.3, disp_size.width*0.5 - border, disp_size.height*0.6);
	return turn;
}

//End of depthmap processing

int main (int argc, char** argv) {

parseArguments(argc, argv);

//Load streamer
Streamer stream(1);

//Open cameras
Camera cap(0);
Camera cap2(1);

//Load stereomodule
StereoCamera stereo;



//Create frames for both cameras
Mat frame;
Mat frame2;

Size frameSize(1280, 720);

//Setting camera resolution
cap.setSize(frameSize.width, frameSize.height);
cap2.setSize(frameSize.width, frameSize.height);

//Setting ROI of depthmap
Rect area(0, 0, frameSize.width, 300);
//Checking cameras
if (!cap.isOpened()) {
	cout << "Couldn't open camera 1 \n" << endl;
	return -1;
}
if (!cap2.isOpened()) {
	cout << "Couldn't open camera 2 \n" << endl;
	return -1;
}

//Object detector module init
Mat frame_detect;
bool target_found = false;
double direction;
double target_size;
if (!object_cascade.load(object_cascade_name)) { printf("classifier cannot be loaded \n"); return -1; }

//Grabbing first frame for further image settings
cap.grab();
cap.retrieve(frame);

//Sliders for camera parameters control
stereo.showMenu();

//Images for depth maps
//Mat disp = Mat::zeros(frame.size(), frame.type());
Mat disp;
Mat disp8;
//Default point for distance measurement
Point2i punkt(300, 300);

// Loading and checking camera settings
if(cap.setIntrinsics("extrinsics.yml", 1) 
	&& cap2.setIntrinsics("extrinsics.yml", 2) 
	&&stereo.setExtrinsics("extrinsics.yml"))

printf("Camera settings have been read properly.\n");
else {
	printf("Problem in reading camera settings. \n");
	return -1;
}

printf("frame size: %d x %d \n",frame.cols, frame.rows);
//Preparing undistortion maps for further frame transformations
cap.setUndistortRectifyMap(frameSize);
cap2.setUndistortRectifyMap(frameSize);

//Initialize robot
RobotControl robot(speed);
//Stream battery voltage
stream.sendVoltage(volt());
//Define default target
Rect target(frameSize.width*0.5*0.2,frameSize.height*0.5,10,10);
//Creating loop counter
int i = 0;
//Default starting key for controlling robot
char cKey = 'w';
double dist = 1.0;

while (true) {
	
	//cap.setExp(-9);
	//cap.setExp(-9);

	cap.grab();
	cap2.grab();
	cap.grab();
	cap2.grab();
	cap.grab();
	cap2.grab();
	cap.grab();
	cap2.grab();
	cap.grab();
	cap2.grab();
	cap.retrieve(frame);
	cap2.retrieve(frame2);

	frame_detect = frame;
	resize(frame_detect, frame_detect, Size(), 0.2, 0.2, INTER_AREA);

	cvtColor(frame, frame, COLOR_BGR2GRAY);
	cvtColor(frame2, frame2, COLOR_BGR2GRAY);

	cap.remapFrame(frame);
	cap2.remapFrame(frame2);
	
	//resize(frame, frame, Size(), 0.2, 0.2, INTER_AREA);
	//resize(frame2, frame2, Size(), 0.2, 0.2, INTER_AREA);

	scan_line1 = frame(area);
	scan_line2 = frame(area);

//	object searching
	//target_found=detectAndDisplay(frame_detect, target);

	//compute depthmap
	stereo.setParams();
	stereo.match(scan_line1, scan_line2, disp);
	//stereo.match(scan_line1, scan_line2, disp);

	disp.convertTo(disp8, CV_8U, 255 / (stereo.numberOfDisparities*16.));

	//resize(disp8, disp8, Size(), 2, 2, INTER_LINEAR);
	Mat preview;

	disp8.convertTo(preview, -1, double(stereo.ratio) / 50., stereo.offset - 200);
//distance from central area
	
	dist = distCentralArea(disp, stereo);

//choosing direction to turn by sides comparison
	turn= avoidDirection(disp);
//showing interface on the disparity image

	applyColorMap(preview, preview, COLORMAP_JET);
	rectangle(preview, area_rect, Scalar(255, 255, 200), 2, 8);

	rectangle(preview, left_area, Scalar(255, 50, 50), 2, 8);
	rectangle(preview, right_area, Scalar(0, 100, 255), 2, 8);
	putText(preview, text, Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 250, 255), 2, CV_AA, 0);
	imshow("disparity", preview);
	
// end of camera setup

	i++;
	direction = (target.x + target.width*0.5 - frame_detect.cols*0.5)/frame_detect.cols;
	target_size = target.width / frame_detect.cols;

// keyboard button press

	int iKey = waitKey(1);
	if (iKey == 27)
	{
		stream.sendStatus(0);
		robot.quit();
		break;
	}
	if (iKey>0 && iKey<255)
	cKey = (char)iKey;

// robot control
	dist = 0.9;
	robot.decide(cKey, direction, dist, turn, target_size);
	//robot.headTo(direction);
	if(!target_found) robot.square();
	robot.move();
	robot.showStatus();

// Odometry
	robot.readEncoders(enc_left, enc_right, enc_l_dir, enc_r_dir);
	decodeEncoders();
	updateCoordinates(enc_diff_left, enc_diff_right);
	//thread t1(updateMap, position);
	updateMap(position);

	reprojectImageTo3D(disp, image3d, stereo.Q);
	map3d(map, image3d);
	imshow("map", background + map + robot_shape);

	//t1.join();
//Streaming
	//if (i%10 == 0) stream.send(posx, posy);

}
return 0;
}