/*
* @file SURF_FlannMatcher
* @brief SURF detector + descriptor + FLANN Matcher
* @author A. Huaman
*/
#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include "FeatureDetection.h"
using namespace std;
using namespace cv;
//using namespace cv::xfeatures2d;
void readme();
/*
* @function main
* @brief Main function
*/
int main(int argc, char** argv)
{
	FeatureDetection feature("iii.PNG",13900);
	VideoCapture cap(1);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	Mat img_2;
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	
	//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
	int minHessian = 3000;
	
	int q = 0;
	while (q < 30) {
		cap.read(img_2);
		imshow("okno", img_2);
		waitKey(20);
		q++;
	}
	while (true) {
	
		cap.grab();
		cap.retrieve(img_2);
		feature.search(img_2);
		waitKey(20);
	}
	return 0;
}
/*
* @function readme
*/
void readme()
{
	std::cout << " Usage: ./SURF_FlannMatcher <img1> <img2>" << std::endl;
}
