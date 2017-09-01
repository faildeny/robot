#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//#include "opencv2/xfeatures2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

using namespace std;
using namespace cv;

class FeatureDetection {
public:
	Mat img_1;
	int slider_dist = 13900;
	int thresh = 2300;
	Rect area;
	Mat img_2;
	int minHessian = 3000;
	//Ptr<SURF> detector = SURF::create();
	Ptr<ORB> detector;
	//detector->setUpright(true);
	//detector->setHessianThreshold(minHessian);
	vector<KeyPoint> keypoints_1, keypoints_2;
	Mat descriptors_1, descriptors_2;
public:
	FeatureDetection(cv::String filename, int dist);
	~FeatureDetection();
	void search(Mat image);
};