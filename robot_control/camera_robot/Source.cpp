#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
extern "C" {
#include "gopigo.h"
}

#include "RobotControl.h"

using namespace cv;
using namespace std;


String object_cascade_name = "cascade.xml";
CascadeClassifier object_cascade;
RNG rng(12345);

void detectAndDisplay(Mat frame, Rect &target)
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
	if (faces.size() == 0)
		target = none;

	imshow("oknno", frame);
}



int main (void) {

VideoCapture cap(0);
VideoCapture cap2(1);

Mat frame;
Mat frame2;
Mat frame_detect;
bool success;
bool success2;

Size frame_size(1280, 720);

cap.set(CV_CAP_PROP_FRAME_HEIGHT, frame_size.height);
cap.set(CV_CAP_PROP_FRAME_WIDTH, frame_size.width);
cap2.set(CV_CAP_PROP_FRAME_HEIGHT, frame_size.height);
cap2.set(CV_CAP_PROP_FRAME_WIDTH, frame_size.width);


if (!cap.isOpened()) {
	cout << "nie moge otworzyc kamery 1" << endl;
	return -1;
}

if (!cap2.isOpened()) {
	cout << "nie moge otworzyc kamery 2" << endl;
	return -1;
}


if (!object_cascade.load(object_cascade_name)) { printf("classifier cannot be loaded \n"); return -1; }

cap.grab();
cap.retrieve(frame);
cap2.grab();
cap2.retrieve(frame2);

//StereoBM::StereoMatcher;
namedWindow("okno");
int numdis = 3; //7
int wsize = 5; //6
int prefilter = 31;
int texturet = 10;
int speckleSize = 200;
int unique = 5;
int dispmax = 1;
int roiw1 = 100;
int roiw2 = 200;
int ratio = 60; //90
int offset = 135; //125
int exposure = 5;
createTrackbar("Liczba dysparycji", "okno", &numdis, 16);
createTrackbar("Rozmiar okna", "okno", &wsize, 40);
createTrackbar("Prefilter", "okno", &prefilter, 100);
createTrackbar("Texture threshold", "okno", &texturet, 50);
createTrackbar("Speckle Window", "okno", &speckleSize, 500);
createTrackbar("Uniqueness ratio", "okno", &unique, 100);
createTrackbar("MinDisp", "okno", &dispmax, 10);
createTrackbar("ROI 1", "okno", &roiw1, 4000);
createTrackbar("ROI 2", "okno", &roiw2, 500);
createTrackbar("Skala", "okno", &ratio, 500);
createTrackbar("Przesuniecie", "okno", &offset, 400);
createTrackbar("Exposure", "okno", &exposure, 15);

Mat disp = Mat::zeros(frame.size(), frame.type());
Mat disp8;
Point2i punkt(300, 300);
Ptr<StereoBM> bm = StereoBM::create(16, 9);

int SADWindowSize = 15;
int numberOfDisparities = 64;

// Camera transformation matrices

Mat CM1 = Mat(3, 3, CV_64FC1);
Mat CM2 = Mat(3, 3, CV_64FC1);
Mat D1, D2;
Mat R, T, E, F;
Mat R1, R2, P1, P2, Q;
Mat map1x, map1y, map2x, map2y;
Mat imgU1, imgU2;

FileStorage fs1("extrinsics.yml", FileStorage::READ);
if (!fs1.isOpened())
{
	cout << "Failed to open " << endl;
	return 1;
}

fs1["M1"] >> CM1;
fs1["M2"] >> CM2;
fs1["D1"] >> D1;
fs1["D2"] >> D2;
fs1["R"] >> R;
fs1["T"] >> T;
fs1["E"] >> E;
fs1["F"] >> F;
fs1["R1"] >> R1;
fs1["R2"] >> R2;
fs1["P1"] >> P1;
fs1["P2"] >> P2;
fs1["Q"] >> Q;

printf("Done Calibration\n");
printf("D1: %d", D1.cols);
printf("Starting Rectification\n");

printf("frame size: %d x %d \n",frame.cols, frame.rows);
printf("CM1: %d x %d \n", CM1.data[0], D1.data[0]);

initUndistortRectifyMap(CM1, D1, R1, P1, frame.size(), CV_32FC1, map1x, map1y);
initUndistortRectifyMap(CM2, D2, R2, P2, frame2.size(), CV_32FC1, map2x, map2y);

printf("map1x size: %d x %d dims: %d area: %d \n", map1x.cols, map1x.rows, map1x.dims, map1x.size().area());
printf("map1y size: %d x %d dims: %d area: %d \n", map1y.cols, map1y.rows, map1y.dims, map1y.size().area());
printf("map2x size: %d x %d dims: %d area: %d \n", map2x.cols, map2x.rows, map2x.dims, map2x.size().area());
printf("map2y size: %d x %d dims: %d area: %d \n", map2y.cols, map2y.rows, map2y.dims, map2y.size().area());

//Initialize robot
RobotControl robot(60);

Rect target(frame_size.width*0.5,frame_size.height*0.5,10,10);
int i = 0;
char cKey = 'w';

while (true) {

	//cap.set(CV_CAP_PROP_EXPOSURE, -exposure);
	//cap2.set(CV_CAP_PROP_EXPOSURE, -exposure);

	success = cap.grab();
	success2 = cap2.grab();
	cap.retrieve(frame);
	cap2.retrieve(frame2);
	frame_detect = frame;
	
	resize(frame_detect, frame_detect, Size(), 0.2, 0.2, INTER_AREA);

	cvtColor(frame, frame, COLOR_BGR2GRAY);
	cvtColor(frame2, frame2, COLOR_BGR2GRAY);
	
	remap(frame, frame, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	remap(frame2, frame2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	resize(frame, frame, Size(), 0.2, 0.2, INTER_AREA);
	resize(frame2, frame2, Size(), 0.2, 0.2, INTER_AREA);
	imshow("kamera 1", frame);
	//imshow("kamera 2", frame2);
	Mat difference = frame - frame2;
	//imshow("Diff", difference);

	detectAndDisplay(frame_detect, target);

	numberOfDisparities = numdis * 16;
	SADWindowSize = wsize * 2 + 1;

	
	bm->setPreFilterCap(prefilter); //31
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(texturet); //10
	bm->setUniquenessRatio(unique); //15
	bm->setSpeckleWindowSize(speckleSize); //200
	bm->setSpeckleRange(32); //32
	bm->setDisp12MaxDiff(dispmax); //1

	bm->compute(frame, frame2, disp);

	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

	//resize(disp8, disp8, Size(), 2, 2, INTER_LINEAR);
	Mat preview;


//distance from central area

	double min, max;
	ostringstream ss;
	disp8.convertTo(preview, -1, double(ratio) / 50., offset - 200);
	Size disp_size = disp.size();

	Rect area_rect(disp_size.width / 2 - disp_size.width / 4, disp_size.height / 2 - disp_size.height / 4, disp_size.width / 2, disp_size.height / 2);
	
	
	Range area_h(disp_size.height/2-disp_size.height / 4, disp_size.height/2+disp_size.height/4);
	Range area_w(disp_size.width / 2 - disp_size.width / 4, disp_size.width / 2 + disp_size.width / 4);
	disp.convertTo(disp, CV_32FC1);
	minMaxLoc(disp(area_h, area_w), &min, &max);
	//float d = disp.at<float>(punkt);
	double distance =0.2 * 0.001 / Q.at<double>(3, 2)*Q.at<double>(2, 3) / max*16.f;
	max = distance;
	ss << max;
	String text = ss.str();

//choosing direction to turn by sides comparison
	int sum_l, sum_r;
	int turn;
	int border=50;
	Range dir_area_l(border, disp_size.width*0.5);
	Range dir_area_r(disp_size.width*0.5, disp_size.width - border);
	Range dir_area_h(disp_size.height*0.3, disp_size.height*0.9);
	Scalar sum_l_scalar = sum(preview(dir_area_h, dir_area_l));
	sum_l = sum_l_scalar[0]/countNonZero(preview(dir_area_h, dir_area_l));
	Scalar sum_r_scalar = sum(preview(dir_area_h, dir_area_r));
	sum_r = sum_r_scalar[0] /countNonZero(preview(dir_area_h, dir_area_r));
	turn = sum_l - sum_r;

	Rect left(border, disp_size.height*0.3, disp_size.width*0.5 - border, disp_size.height*0.6);
	Rect right(disp_size.width*0.5, disp_size.height*0.3, disp_size.width*0.5 - border, disp_size.height*0.6);

//showing interface on the disparity image

	applyColorMap(preview, preview, COLORMAP_JET);
	rectangle(preview, area_rect, Scalar(255, 255, 200), 2, 8);

	rectangle(preview, left, Scalar(255, 50, 50), 2, 8);
	rectangle(preview, right, Scalar(0, 100, 255), 2, 8);
	putText(preview, text, Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 250, 255), 2, CV_AA, 0);
	imshow("disparity", preview);

// end of camera setup

	i++;
	//dst = us_dist(15);
	double direction = (target.x + target.width*0.5 - frame_detect.cols*0.5)/frame_detect.cols;

// keyboard button press

	int iKey = waitKey(1);
	if (iKey == 27)
	{
		break;
	}
	if (iKey>0 && iKey<255)
	cKey = (char)iKey;
	cout << iKey;
// robot control

	robot.decide(cKey, direction, distance, turn);
	//robot.headTo(direction);
	//robot.square();
	robot.showStatus();
	robot.move();

}
return 0;
}