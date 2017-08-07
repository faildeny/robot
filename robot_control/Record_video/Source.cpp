#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;


void saveImage(Mat frame, int n) {
	String filename("frame/image_");
	filename += to_string(n);
	filename += ".jpg";
	imwrite(filename, frame);
}


int main() {

	VideoCapture cap(0);

	Mat frame;

	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	int n = 1;

	while (true) {
		cap.read(frame);
		saveImage(frame, n++);
		imshow("frame", frame);

		int key = waitKey(1);
		if (key == 27) {
			break;
		}
	}
	return 0;
}