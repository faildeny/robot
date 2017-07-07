#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

int main (void) {

VideoCapture cap(0);
VideoCapture cap2(1);

Mat frame;
Mat frame2;
bool success;
bool success2;

cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);
cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
cap2.set(CV_CAP_PROP_FRAME_HEIGHT, 360);
cap2.set(CV_CAP_PROP_FRAME_WIDTH, 640);


if (!cap.isOpened()) {
	cout << "nie moge otworzyc kamery 1" << endl;
	return -1;
}

if (!cap2.isOpened()) {
	cout << "nie moge otworzyc kamery 2" << endl;
	return -1;
}





//StereoBM::StereoMatcher;
namedWindow("okno");
int numdis = 3; //7
int wsize = 5; //6
int prefilter = 31;
int texturet = 10;
int speckleSize = 200;
int unique = 15;
int dispmax = 1;
int roiw1 = 100;
int roiw2 = 200;
int ratio = 159; //90
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


while (true) {

	cap.set(CV_CAP_PROP_EXPOSURE, -exposure);
	cap2.set(CV_CAP_PROP_EXPOSURE, -exposure);

	success = cap.grab();
	success2 = cap2.grab();
	cap.retrieve(frame);
	cap2.retrieve(frame2);

	resize(frame, frame, Size(), 0.4, 0.4, INTER_AREA);
	cvtColor(frame, frame, COLOR_BGR2GRAY);
	resize(frame2, frame2, Size(), 0.4, 0.4, INTER_AREA);
	cvtColor(frame2, frame2, COLOR_BGR2GRAY);

	//resize(frame, frame, Size(), 0.4, 0.4, INTER_AREA);
	imshow("kamera 1", frame);
	imshow("kamera 2", frame2);


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
	double *min, *max;
	ostringstream ss;
	disp8.convertTo(preview, -1, double(ratio) / 50., offset - 200);

	Size rozmiar = preview.size();
	Range area(rozmiar.height/2-rozmiar.height / 4, rozmiar.height/2+rozmiar.height/4);
	minMaxLoc(preview(area, area),min,max);
	ss << *max;
	String text = ss.str();
	putText(preview, text, punkt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 250, 255), 2, CV_AA, 0);

	applyColorMap(preview, preview, COLORMAP_JET);
	imshow("disparity", preview);

	int iKey = waitKey(50);
	if (iKey == 27)
	{
		break;
	}
}
return 0;
}