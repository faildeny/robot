#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int speed = 60;

//void saveImage(Mat frame, int n) {
//	String filename("frame/image_");
//	filename += to_string(n);
//	filename += ".jpg";
//	imwrite(filename, frame);
//}


int main(int argc, char** argv) {

	cout << argc << endl;
	for (int i = 0; i < argc; i++) {
		cout << "argv[i]= " << argv[i] << endl;
		if (string(argv[i]) == "-v" && i+1<argc) {
			speed = atoi(argv[i + 1]);
			cout << "speed: " << speed << endl;
		}
	}

	VideoCapture cap(1);
	VideoCapture cap1(2);

	Mat frame;
	Mat frame1;
	Mat diff;

	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap1.set(CAP_PROP_FRAME_HEIGHT, 720);
	cap1.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_EXPOSURE, -11);
	cap1.set(CAP_PROP_EXPOSURE,-11);

	int n = 1;

	while (true) {
		cap.grab();
		cap.retrieve(frame);
		cap1.grab();
		cap1.retrieve(frame1);
	/*	cap.grab();
		cap1.grab();
		cap.grab();
		cap1.grab();
		cap.grab();
		cap1.grab();
		cap.grab();
		cap1.grab();*/
		resize(frame, frame, Size(), 0.2, 0.2);
		resize(frame1, frame1, Size(), 0.2, 0.2);
		//diff = frame*0.5 + frame1*0.5;
		//saveImage(frame, n++);
		imshow("frame", frame);
		imshow("frame1", frame1);
		//imshow("diff", diff);

		int key = waitKey(1);
		if (key == 27) {
			saveImage(frame, n);
			saveImage(frame1, n + 1);
			break;
		}
	}
	return 0;
}