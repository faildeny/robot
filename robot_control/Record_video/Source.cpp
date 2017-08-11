#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int speed = 60;

void saveImage(Mat frame, int n) {
	String filename("frame/image_");
	filename += to_string(n);
	filename += ".jpg";
	imwrite(filename, frame);
}


int main(int argc, char** argv) {

	cout << argc << endl;
	for (int i = 0; i < argc; i++) {
		cout << "argv[i]= " << argv[i] << endl;
		if (string(argv[i]) == "-v" && i+1<argc) {
			speed = atoi(argv[i + 1]);
			cout << "speed: " << speed << endl;
		}
	}

	VideoCapture cap(0);

	Mat frame;

	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	int n = 1;

	while (true) {
		cap.read(frame);
		//saveImage(frame, n++);
		imshow("frame", frame);

		int key = waitKey(1);
		if (key == 27) {
			break;
		}
	}
	return 0;
}