#pragma once
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

#define MAX_FRAME 4000
#define MIN_NUM_FEAT 400

class VisualOdometry {

private:
	Mat img_1, img_2;
	Mat R_f, t_f; //the final rotation and tranlation vectors containing the 
	double scale = 2;

	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale;
	int thickness;
	Point textOrg;

	Mat img_1_c;
	Mat img_2_c;
	vector<Point2f> points1, points2;
	vector<uchar> status;
	double focal;
	//cv::Point2d pp(644.09296, 348.5013);
	Point2d pp;
	Mat E, R, t, mask;
	Mat prevImage;
	Mat currImage;
	vector<Point2f> prevFeatures;
	vector<Point2f> currFeatures;
	Mat traj;

public:
	VisualOdometry(Mat image,double scale_param);

	~VisualOdometry();

	void initOdometry(Mat image);

	void update(Mat image);

private:

	void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);

	void featureDetection(Mat img_1, vector<Point2f>& points1);

};