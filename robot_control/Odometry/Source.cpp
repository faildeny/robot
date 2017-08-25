#include "VisualOdometry.h"

#include <iostream>
#include <stdio.h>

#include <chrono>
#include <thread>
using namespace std::chrono;
//#include "initial-state.h"

#define _CRT_SECURE_NO_WARNINGS

void capture(VideoCapture cap, Mat frame) {
	cap.grab();
	cap.retrieve(frame);
}
int main() {
	VideoCapture cap(0);
	Mat image;
	Mat image2;
	cap.grab();
	cap.retrieve(image);
	
	
	//VisualOdometry vis_odo(image,2.0);

	printf("HEllo");


	
		


	while (true) {

		high_resolution_clock::time_point t1 = high_resolution_clock::now();
		thread thread1(capture, cap, image);
		//cap.grab();
		
		//vis_odo.update(image);
		imshow("frame", image);
		waitKey(1);
		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(t2 - t1).count();
		cout << "elapsed time: " << (double)duration / 1000 << " ms" << endl;
		thread1.join();
	}
	return 0;
}