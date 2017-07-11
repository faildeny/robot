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

using namespace cv;
using namespace std;


String face_cascade_name = "haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;
RNG rng(12345);

void detectAndDisplay(Mat frame, Rect &target)
{
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
	for (int i = 0; i < faces.size(); i++)
	{
		rectangle(frame, faces[i], Scalar(0, 255, 200), 2, 8);
		target = faces[0];
	}
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

Size frame_size(320, 180);

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


if (!face_cascade.load(face_cascade_name)) { printf("classifier cannot be loaded \n"); return -1; }


//StereoBM::StereoMatcher;
namedWindow("okno");
int numdis = 4; //7
int wsize = 7; //6
int prefilter = 31;
int texturet = 10;
int speckleSize = 200;
int unique = 5;
int dispmax = 1;
int roiw1 = 100;
int roiw2 = 200;
int ratio = 80; //90
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
Rect roi1(1, 1, 1000, 1000);
Rect roi2(1, 1, roiw2, roiw2);

//Initialize robot

if (init() == -1) {
	exit(1);
}

char cmd[3];
cmd[0] = 'w';
cmd[1] = 'x';
cmd[2] = 'd';
set_speed(100);

int i = 0;
int dst = 0;
Rect target(frame_size.width*0.5,frame_size.height*0.5,10,10);

///////

while (true) {

	//cap.set(CV_CAP_PROP_EXPOSURE, -exposure);
	//cap2.set(CV_CAP_PROP_EXPOSURE, -exposure);

	success = cap.grab();
	success2 = cap2.grab();
	cap.retrieve(frame);
	cap2.retrieve(frame2);
	frame_detect = frame;
	detectAndDisplay(frame_detect, target);

	resize(frame, frame, Size(), 0.4, 0.4, INTER_AREA);
	cvtColor(frame, frame, COLOR_BGR2GRAY);
	resize(frame2, frame2, Size(), 0.4, 0.4, INTER_AREA);
	cvtColor(frame2, frame2, COLOR_BGR2GRAY);

	imshow("kamera 1", frame);
	//imshow("kamera 2", frame2);
	Mat difference = frame - frame2;
	imshow("Diff", difference);

	numberOfDisparities = numdis * 16;
	SADWindowSize = wsize * 2 + 1;

	bm->setROI1(roi1);
	bm->setROI2(roi2);
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

	resize(disp8, disp8, Size(), 3, 3, INTER_LINEAR);
	Mat preview;
	double min, max;
	ostringstream ss;
	disp8.convertTo(preview, -1, double(ratio) / 50., offset - 200);
	Size rozmiar = preview.size();

	Rect area_rect(rozmiar.width / 2 - rozmiar.width / 4, rozmiar.height / 2 - rozmiar.height / 4, rozmiar.width / 2, rozmiar.height / 2);
	
	
	Range area(rozmiar.height/2-rozmiar.height / 4, rozmiar.height/2+rozmiar.height/4);
	minMaxLoc(preview(area, area),&min,&max);
	ss << max;
	String text = ss.str();
	

	//applyColorMap(preview, preview, COLORMAP_JET);
	//rectangle(preview, area_rect, Scalar(255, 255, 200), 2, 8);
	//putText(preview, text, Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 250, 255), 2, CV_AA, 0);
	//imshow("disparity", preview);

	////////////////////////////////////

	i++;
	if (i > 200)
	{
		cmd[0] = 'x';
		stop();
		led_off(0);
		led_off(1);
		exit(0);
		break;
	}

	switch (cmd[0])
	{
	case 'w':

		printf("mozna jechac: ");
		switch (cmd[1])
		{
		case 'w':
			switch (cmd[2])
			{
			case 'd':
				printf("skrecam w prawo");
				led_on(1);
				led_off(0);
				set_speed(100);
				right_rot();
				break;
			case 'a':
				printf("skrecam w lewo");
				led_on(0);
				led_off(1);
				set_speed(100);
				left_rot();
				break;
			case 'w':
				printf("jade");
				led_off(0);
				led_off(1);
				//motor1(1, 30);
				//motor2
				set_speed(80);
				fwd();
				break;
			}

		case 'r':

			switch (cmd[2])
			{	
			case 'd':
				printf("skrecam w prawo");
				led_on(1);
				led_off(0);
				set_speed(40);
				right_rot();
				break;
			case 'a':
				printf("skrecam w lewo");
				led_on(0);
				led_off(1);
				set_speed(40);
				left_rot();
				break;
			}
		}

		break;

	case 'x':
		stop();
		break;

	}
	dst = us_dist(15);
	int direction = target.x + target.width*0.5 - frame_size.width*0.5;
	//cmd[1] = (dst > 40) ? 'w' : 'd';
	cmd[1] = (max < 170 && max > 0) ? 'w' : 'r';
	if (direction < 10 && direction > -10) {
		cmd[2] = 'w';
	}
	else 
	{ 
		cmd[2] = (direction < -10) ? 'a' : 'd';
	}
	
	printf(" i= %d command= %c %c %c \n", i, cmd[0], cmd[1],cmd[2]);
	printf("dystans: %d\n", dst);

	////////////////////////////////

	int iKey = waitKey(20);
	if (iKey == 27)
	{
		break;
	}
	if (iKey == 32)
	{
		cmd[0] = 'x';
	}
	if (iKey == 119)
	{
		cmd[0] = 'w';
	}
}
return 0;
}