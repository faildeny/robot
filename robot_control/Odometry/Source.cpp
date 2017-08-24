#include "VisualOdometry.h"

#include <iostream>
#include <stdio.h>
//#include "initial-state.h"

#define _CRT_SECURE_NO_WARNINGS


int main() {
	VideoCapture cap(0);
	Mat image;
	Mat image2;
	cap.grab();
	cap.retrieve(image);
	
	
	VisualOdometry vis_odo(image,2.0);

	printf("HEllo");


	
		


	while (true) {


		cap.grab();
		cap.retrieve(image);
		vis_odo.update(image);

		waitKey(10);
	}
	return 0;
}