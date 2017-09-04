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
#include <mutex>
#include <cstring>
#include <pthread.h>

extern "C" {
#include "gopigo.h"
}
#include "RobotOdometry.h"
#include "FeatureDetection.h"
#include "VisualOdometry.h"
#include "StereoCamera.h"
#include "Streamer.h"
#include "RobotControl.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

mutex iomutex;


String object_cascade_name = "cascade.xml";
CascadeClassifier object_cascade;
RNG rng(12345);

int speed=60;

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

void parallelGrab(Camera cap, Mat *frame,int priority,int n) {
	high_resolution_clock::time_point time1 = high_resolution_clock::now();

	sched_param sch;
	int policy;
	pthread_getschedparam(pthread_self(), &policy, &sch);
	//std::lock_guard<std::mutex> lk(iomutex);

	if (priority >= 0) {
		sch.sched_priority = priority;
		if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch)) {
			std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
		}
	}
	for (int i = 0; i < n; i++) {
		cap.grab();	
	}
	cap.retrieve(*frame);

	high_resolution_clock::time_point time2 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(time2 - time1).count();
	cout << "grab thread executed in: " << (double)duration2 / 1000 << " ms" << endl;
}

void parallelRemap(Camera cap, Mat *frame, double scale,int priority) {
	high_resolution_clock::time_point time1 = high_resolution_clock::now();

	sched_param sch;
	int policy;
	pthread_getschedparam(pthread_self(), &policy, &sch);
	//std::lock_guard<std::mutex> lk(iomutex);

	if (priority >= 0) {
		sch.sched_priority = priority;
		if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch)) {
			std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
		}
	}
	cvtColor(*frame, *frame, COLOR_BGR2GRAY);
	resize(*frame, *frame, Size(), scale, scale, INTER_AREA);
	cap.remapFrame(*frame);
	

	high_resolution_clock::time_point time2 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(time2 - time1).count();
	cout << "remapping thread executed in: " << (double)duration2 / 1000 << " ms" << endl;
}
void parallelRemap2(Camera cap, Mat *frame, double scale, int priority) {
	high_resolution_clock::time_point time1 = high_resolution_clock::now();

	sched_param sch;
	int policy;
	pthread_getschedparam(pthread_self(), &policy, &sch);
	//std::lock_guard<std::mutex> lk(iomutex);

	if (priority >= 0) {
		sch.sched_priority = priority;
		if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch)) {
			std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
		}
	}

	cap.remapFrame(*frame);
	resize(*frame, *frame, Size(), scale, scale, INTER_AREA);

	high_resolution_clock::time_point time2 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(time2 - time1).count();
	cout << "cam 2 remapping  thread executed in: " << (double)duration2 / 1000 << " ms" << endl;
}

void parallelCam(Camera cap, Mat *frame, double scale, int priority) {
	high_resolution_clock::time_point time1 = high_resolution_clock::now();
	
	sched_param sch;
	int policy;
	pthread_getschedparam(pthread_self(), &policy, &sch);
	//std::lock_guard<std::mutex> lk(iomutex);
	
	if (priority >= 0) {
		sch.sched_priority = priority;
			if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch)) {
				std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
			}
		}
	//std::cout << "ThreadCam " << " is executing at priority "
	//	<< sch.sched_priority << '\n';

	cap.grab();
	cap.grab();
	cap.grab();
	cap.grab();
	cap.grab();
	cap.retrieve(*frame);

	
	resize(*frame, *frame, Size(), scale, scale, INTER_AREA);
	cap.remapFrame(*frame);
	

	high_resolution_clock::time_point time2 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(time2 - time1).count();
	//cout << "camera "<<priority<<" thread executed in: " << (double)duration2 / 1000 << " ms" << endl;
}

void parallelOdometry(Mat image, VisualOdometry vis_odo) {
	vis_odo.update(image);
}

void parallelFeature(FeatureDetection feature, Mat frame_detect, bool *target_found_p) {
	*target_found_p=feature.search(frame_detect);
}
void parallelMapping(RobotOdometry* odometry) {
	odometry->decodeEncoders();
	odometry->updateCoordinates();
	////thread t1(updateMap, position);
	odometry->updateMap();
	//
	//disp.convertTo(disp, CV_32F);
	//reprojectImageTo3D(disp, image3d, stereo.Qs);
	////customReproject(disp, stereo.Qs, image3d);
	//map3d(map, image3d);
	imshow("map", odometry->background + odometry->map + odometry->robot_shape);
}

void f(int num)
{
	//std::this_thread::sleep_for(std::chrono::seconds(1));
	high_resolution_clock::time_point time1 = high_resolution_clock::now();
	int priority = 99;
	sched_param sch;
	int policy;
	pthread_getschedparam(pthread_self(), &policy, &sch);
	//std::lock_guard<std::mutex> lk(iomutex);
	if (priority >= 0) {
		sch.sched_priority = priority;
		if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch)) {
			std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
		}
	}
	std::cout << "Thread " << num << " is executing at priority "
		<< sch.sched_priority << '\n';
	double a = 1.0;
	for (int i = 0; i < 10000;i++) {
		a = (a + (double)i) / 3;
	}

	high_resolution_clock::time_point time2 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(time2 - time1).count();
	cout << "task " << num << " executed in: " << (double)duration2 / 1000 << " ms" << endl;
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

//Load odometry module
RobotOdometry odometry;
RobotOdometry* odometry_p = &odometry;

//Load ORB feature detector
FeatureDetection feature("iii.png", 13900);

//Create frames for both cameras
Mat frame;
Mat frame2;
Mat temp1;
Mat temp2;

Mat *framep;
framep = &frame;
Mat *framep2;
framep2 = &frame2;



Size frameSize(1280, 720);
double scale = 0.2;
Size frameSizeScaled(frameSize.width*scale, frameSize.height*scale);

//Setting camera resolution
cap.setSize(frameSize.width, frameSize.height);
cap2.setSize(frameSize.width, frameSize.height);

//Turning off auto exposure
//cap.set(CAP_PROP_AUTO_EXPOSURE, 1);
//cap2.set(CAP_PROP_AUTO_EXPOSURE, 1);

//Setting ROI of depthmap
Rect area(0, 0, frameSize.width*0.2, 130);
Mat scan_line1, scan_line2;

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
int far = 0;
Mat frame_detect;
bool target_found = false;
bool *target_found_p = &target_found;
double direction;
double target_size;
if (!object_cascade.load(object_cascade_name)) { printf("classifier cannot be loaded \n"); return -1; }

//Grabbing first frame for further image settings
parallelGrab(cap, framep, 99,6);
parallelGrab(cap2, framep2, 99,6);
//cap.grab();
//cap2.grab();
//cap.retrieve(frame);
//cap2.retrieve(frame2);
cap.retrieve(temp1);
cap2.retrieve(temp2);
Mat image_odo;
frame.copyTo(image_odo);
resize(image_odo, image_odo, Size(), 0.2, 0.2, INTER_AREA);
//cvtColor(image_odo, image_odo, COLOR_BGR2GRAY);
cout << "init Odometry" << endl;
//VisualOdometry vis_odo(image_odo, 2.0);
cout << "odometry initiated" << endl;

//Sliders for camera parameters control
//stereo.showMenu();
//namedWindow("Depth map", CV_WINDOW_KEEPRATIO);

//Images for depth maps
//Mat disp = Mat::zeros(frame.size(), frame.type());
Mat disp;
Mat disp8;
//Default point for distance measurement
Point2i punkt(300, 300);

// Loading and checking camera settings
if(cap.setIntrinsics("extrinsics.yml", 1)
	&& cap2.setIntrinsics("extrinsics.yml", 2) 
	&&stereo.setExtrinsics("extrinsics.yml",scale))

printf("Camera settings have been read properly.\n");
else {
	printf("Problem in reading camera settings. \n");
	return -1;
}

printf("frame size: %d x %d \n",frame.cols, frame.rows);
//Preparing undistortion maps for further frame transformations
cap.scaleIntrinsics(scale);
cap2.scaleIntrinsics(scale);
cap.setUndistortRectifyMap(frameSizeScaled);
cap2.setUndistortRectifyMap(frameSizeScaled);

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

int priority = 1;
sched_param sch5;
int policy5;
pthread_getschedparam(pthread_self(), &policy5, &sch5);
//std::lock_guard<std::mutex> lk(iomutex);

if (priority >= 0) {
	sch5.sched_priority = priority;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch5)) {
		std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
	}
}
cout << "mainThread " << " is executing at priority " << sch5.sched_priority << '\n';

while (true) {


	high_resolution_clock::time_point time1 = high_resolution_clock::now();

	robot.readEncoders(odometry.enc_left, odometry.enc_right, odometry.enc_l_dir, odometry.enc_r_dir);
	thread thread_map(parallelMapping, odometry_p);

	thread t2(parallelGrab, cap2, framep2, 99, 5);
	thread t1(parallelGrab, cap, framep, 99, 5);
	t2.join();
	t1.join();

	frame_detect = frame;
	thread thread_feature(parallelFeature, feature, frame_detect, target_found_p);
	//cascade object detection
	//target_found=detectAndDisplay(frame_detect, target);

	//resize(frame_detect, frame_detect, Size(), 0.2, 0.2, INTER_AREA);
	
	//visual odometry
	//thread visual_odometry(parallelOdometry, frame,vis_odo);
	//if(i==14) vis_odo.initOdometry(frame);
	//if (i>14) vis_odo.update(frame);
	//imshow("frame", frame);

	high_resolution_clock::time_point time2 = high_resolution_clock::now();
	auto duration1 = duration_cast<microseconds>(time2 - time1).count();
	cout << "grabing threaded: " << (double)duration1 / 1000 << " ms" << endl;

	thread t4(parallelRemap, cap, framep, scale, 98);
	thread t3(parallelRemap, cap2, framep2, scale, 98);
	t3.join();
	t4.join();

	/*resize(frame, frame, Size(), 0.2, 0.2, INTER_AREA);
	resize(frame2, frame2, Size(), 0.2, 0.2, INTER_AREA);*/
	//temp1.copyTo(frame);
	//temp2.copyTo(frame2);

	high_resolution_clock::time_point time3 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(time3 - time2).count();
	
	cout << "remapping threaded in: " << (double)duration2 / 1000 << " ms" << endl;

	
	

	high_resolution_clock::time_point time9 = high_resolution_clock::now();
	auto duration9 = duration_cast<microseconds>(time9 - time3).count();
	cout << "feature detection: " << (double)duration9 / 1000 << " ms" << endl;

	
	scan_line1 = frame(area);
	scan_line2 = frame2(area);
	
	
	//set parameters and compute depthmap
	stereo.setParams();
	stereo.match(scan_line1, scan_line2);

	//stereo.preparePreview();
	
	//showing interface on the disparity image
	//stereo.drawDashboard();

	//compute distance from central area	
	dist = stereo.distCentralArea();

	//analize depthmap for choosing movement direction
	turn= stereo.avoidDirection();

	//imshow("Depth map", stereo.preview);
	
	high_resolution_clock::time_point time4 = high_resolution_clock::now();
	auto duration3 = duration_cast<microseconds>(time4 - time9).count();
	cout << "stereo: " << (double)duration3 / 1000 << " ms" << endl;
// end of camera setup

	//visual_odometry.join();

// Odometry
	
	//odometry.decodeEncoders();
	//odometry.updateCoordinates();
	//////thread t1(updateMap, position);
	//odometry.updateMap();
	////
	////disp.convertTo(disp, CV_32F);
	////reprojectImageTo3D(disp, image3d, stereo.Qs);
	//////customReproject(disp, stereo.Qs, image3d);
	////map3d(map, image3d);
	//imshow("map", odometry.background + odometry.map + odometry.robot_shape);
////

	i++;
	//direction = (target.x + target.width*0.5 - frame_detect.cols*0.5)/frame_detect.cols;
	direction = 0.0;
	target_size = target.width / frame_detect.cols;
	target_size = 0.1;
	
	
	high_resolution_clock::time_point time5 = high_resolution_clock::now();
	auto duration4 = duration_cast<microseconds>(time5 - time4).count();
	cout << "odometry processing: " << (double)duration4 / 1000 << " ms" << endl;

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
	//if(!target_found) robot.square();
	robot.move();
	robot.showStatus();

//Streaming
	//if (i%10 == 0) stream.send(odometry.posx, odometry.posy);

	//target_found = feature.search(frame_detect);
	thread_map.join();
	thread_feature.join();
	if (target_found&& far == 0) {
		cout << "markingTarget" << endl;
		odometry.markTarget();
		far = 10;
	}
	if (far != 0) far--;

	high_resolution_clock::time_point time6 = high_resolution_clock::now();
	auto duration5 = duration_cast<microseconds>(time6 - time5).count();
	cout << "robot functions: " << (double)duration5 / 1000 << " ms" << endl;
	
	
	auto duration = duration_cast<microseconds>(time6 - time1).count();
	cout << "TOTAL TIME: " << (double)duration / 1000 << " ms" << endl;

	//imshow("fr", frame);
	waitKey(1);

}
return 0;
}