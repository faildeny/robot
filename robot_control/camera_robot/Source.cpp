#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main (void) {

VideoCapture cap(0);
VideoCapture cap2(1);

Mat frame;
Mat frame2;
bool success;

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



while (true) {

	cap.set(CV_CAP_PROP_EXPOSURE, -11);
	cap2.set(CV_CAP_PROP_EXPOSURE, -11);

	success = cap.grab();
	success = cap2.grab();
	cap.retrieve(frame);
	cap2.retrieve(frame);

	//resize(frame, frame, Size(), 0.4, 0.4, INTER_AREA);
	imshow("kamera 1", frame);
	imshow("kamera 2", frame2);

	int iKey = waitKey(50);
	if (iKey == 27)
	{
		break;
	}
}
return 0;
}