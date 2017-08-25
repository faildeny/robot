#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <thread>
#include <chrono>

extern "C" {
#include "gopigo.h"
}
#include "VisualOdometry.h"
#include "StereoCamera.h"
#include "Streamer.h"
#include "RobotControl.h"

using namespace cv;
using namespace std;
using namespace std::chrono;


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
int view_angleDeg = 30;
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
	map = Mat::zeros(map.cols, map.rows, map.type());
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
	int j = 120;
	//map = Mat::zeros(600, 1280, CV_8UC3);
	//Rect robot_rect(center_x - 10, center_y - 10, 20, 30);
	//rectangle(map, robot_rect, Scalar(30, 255, 60), 2);
	for (int j = 50; j < 51; j++) {
		for (int i = 20; i < image3d.cols-20; i++) {

			/*int x = int(5 * image3d.at<Vec3f>(j, i)[0]) + center_x);
			int y = int(-5 * image3d.at<Vec3f>(j, i)[2]) + center_y;*/

			int x1 = ((3 * image3d.at<Vec3f>(j, i)[0]));
			int y1 = (-3 * image3d.at<Vec3f>(j, i)[2]);
			//cout << "zaraz dodam " <<y1<<" "<<x1<< endl;
			if (x1!= 0.0&& cos(atan(y1 / x1))!= 0.0) {
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
// Reproject image to 3D
void customReproject(const cv::Mat& disparity, const cv::Mat& Q, cv::Mat& out3D)
{
	CV_Assert(disparity.type() == CV_32F && !disparity.empty());
	CV_Assert(Q.type() == CV_32F && Q.cols == 4 && Q.rows == 4);

	// 3-channel matrix for containing the reprojected 3D world coordinates
	out3D = cv::Mat::zeros(disparity.size(), CV_32FC3);

	// Getting the interesting parameters from Q, everything else is zero or one
	float Q03 = Q.at<float>(0, 3);
	float Q13 = Q.at<float>(1, 3);
	float Q23 = Q.at<float>(2, 3);
	float Q32 = Q.at<float>(3, 2);
	float Q33 = Q.at<float>(3, 3);

	// Transforming a single-channel disparity map to a 3-channel image representing a 3D surface
	for (int i = 0; i < disparity.rows; i++)
	{
		const float* disp_ptr = disparity.ptr<float>(i);
		cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

		for (int j = 0; j < disparity.cols; j++)
		{
			const float pw = 1.0f / (disp_ptr[j] * Q32 + Q33);

			cv::Vec3f& point = out3D_ptr[j];
			point[0] = (static_cast<float>(j) + Q03) * pw;
			point[1] = (static_cast<float>(i) + Q13) * pw;
			point[2] = Q23 * pw;
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

//threading

void parallelGrab(VideoCapture cap) {
	//cap.grab();
	cap.grab();
	cap.grab();
	cap.grab();
	cap.grab();
	cap.grab();
}

void parallelOdometry(Mat image, VisualOdometry vis_odo) {
	vis_odo.update(image);
}

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

//Turning off auto exposure
//cap.set(CAP_PROP_AUTO_EXPOSURE, 1);
//cap2.set(CAP_PROP_AUTO_EXPOSURE, 1);

//Setting ROI of depthmap
Rect area(0, 40, frameSize.width*0.2, 30);
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
Mat image_odo;
frame.copyTo(image_odo);
resize(image_odo, image_odo, Size(), 0.2, 0.2, INTER_AREA);
//cvtColor(image_odo, image_odo, COLOR_BGR2GRAY);
cout << "init Odometry" << endl;
VisualOdometry vis_odo(image_odo, 2.0);
cout << "odometry initiated" << endl;
//Sliders for camera parameters control
stereo.showMenu();
namedWindow("Depth map", CV_WINDOW_KEEPRATIO);

//Images for depth maps
//Mat disp = Mat::zeros(frame.size(), frame.type());
Mat disp;
Mat disp8;
//Default point for distance measurement
Point2i punkt(300, 300);

// Loading and checking camera settings
if(cap.setIntrinsics("extrinsics.yml", 1) 
	&& cap2.setIntrinsics("extrinsics.yml", 2) 
	&&stereo.setExtrinsics("extrinsics.yml",0.2))

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
	
	
	high_resolution_clock::time_point time1 = high_resolution_clock::now();
	//cap.setExp(stereo.exposure);
	//cap2.setExp(-stereo.exposure);
	
	thread t1(parallelGrab, cap);
	thread t2(parallelGrab, cap2);
	t1.join();
	t2.join();

	cap.retrieve(frame);
	cap2.retrieve(frame2);
	
	high_resolution_clock::time_point time2 = high_resolution_clock::now();
	auto duration1 = duration_cast<microseconds>(time2 - time1).count();
	cout << "grab and retrieve: " << (double)duration1 / 1000 << " ms" << endl;
	//Visual odometry

	
	
	frame_detect = frame;
	resize(frame_detect, frame_detect, Size(), 0.2, 0.2, INTER_AREA);

	
	cap.remapFrame(frame);
	cap2.remapFrame(frame2);
	
	resize(frame, frame, Size(), 0.2, 0.2, INTER_AREA);
	resize(frame2, frame2, Size(), 0.2, 0.2, INTER_AREA);
	
	high_resolution_clock::time_point time3 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(time3 - time2).count();
	cout << "remap and resize: " << (double)duration2 / 1000 << " ms" << endl;

	//thread visual_odometry(parallelOdometry, frame,vis_odo);
	if(i==14) vis_odo.initOdometry(frame);
	//if (i>14) vis_odo.update(frame);
	//imshow("frame", frame);
	cvtColor(frame, frame, COLOR_BGR2GRAY);
	cvtColor(frame2, frame2, COLOR_BGR2GRAY);

	Mat diff=frame-frame2;
	//imshow("camera 0", frame);
	//imshow("camera 1", frame2);
	//imshow("Diff", diff);

	scan_line1 = frame(area);
	scan_line2 = frame2(area);

//	object searching
	//target_found=detectAndDisplay(frame_detect, target);

	//compute depthmap
	stereo.setParams();
	//stereo.match(scan_line1, scan_line2, disp);
	//stereo.match(frame, frame2, disp);

	//resize(disp, disp, Size(), 5, 5, INTER_AREA);
	//disp*= 5;
	if (i == 15) {
		FileStorage map("depth_low.xml", FileStorage::WRITE);
		map<<"Depth"<< disp;
		cout << "saved" << endl;
	}
	//imwrite("depth_full.bmp", disp);

	high_resolution_clock::time_point time4 = high_resolution_clock::now();
	auto duration3 = duration_cast<microseconds>(time4 - time3).count();
	cout << "cropping and BGR to GRAY: " << (double)duration3 / 1000 << " ms" << endl;
	stereo.match(scan_line1, scan_line2, disp);

	//disp.convertTo(disp8, CV_8U, 255 / (stereo.numberOfDisparities*16.));

	//resize(disp8, disp8, Size(), 2, 2, INTER_LINEAR);
	//Mat preview;

	//disp8.convertTo(preview, -1, double(stereo.ratio) / 50., stereo.offset - 200);
//distance from central area
	
	dist = distCentralArea(disp, stereo);

//choosing direction to turn by sides comparison
	turn= avoidDirection(disp);

	high_resolution_clock::time_point time5 = high_resolution_clock::now();
	auto duration4 = duration_cast<microseconds>(time5 - time4).count();
	cout << "matching and depthmap processing: " << (double)duration4 / 1000 << " ms" << endl;
//showing interface on the disparity image

	/*applyColorMap(preview, preview, COLORMAP_JET);
	rectangle(preview, area_rect, Scalar(255, 255, 200), 2, 8);

	rectangle(preview, left_area, Scalar(255, 50, 50), 2, 8);
	rectangle(preview, right_area, Scalar(0, 100, 255), 2, 8);
	putText(preview, text, Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 250, 255), 2, CV_AA, 0);
	imshow("Depth map", preview);*/
	
// end of camera setup
	//visual_odometry.join();
// Odometry
	//robot.readEncoders(enc_left, enc_right, enc_l_dir, enc_r_dir);
	//decodeEncoders();
	//updateCoordinates(enc_diff_left, enc_diff_right);
	////thread t1(updateMap, position);
	//updateMap(position);
	//
	//disp.convertTo(disp, CV_32F);
	//reprojectImageTo3D(disp, image3d, stereo.Qs);
	////customReproject(disp, stereo.Qs, image3d);
	//map3d(map, image3d);
	//imshow("map", background + map + robot_shape);
////

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

//Streaming
	//if (i%10 == 0) stream.send(posx, posy);

	high_resolution_clock::time_point time6 = high_resolution_clock::now();
	auto duration5 = duration_cast<microseconds>(time6 - time5).count();
	cout << "robot functions: " << (double)duration5 / 1000 << " ms" << endl;

	auto duration = duration_cast<microseconds>(time6 - time1).count();
	cout << "TOTAL TIME: " << (double)duration / 1000 << " ms" << endl;



}
return 0;
}