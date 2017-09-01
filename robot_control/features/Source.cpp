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
	namedWindow("controls");
	int slider_dist = 2397;
	int thresh = 2300;
	int blurs = 0;
	createTrackbar("dist", "controls", &slider_dist, 50000);
	createTrackbar("thresh", "controls", &thresh, 5000);
	createTrackbar("blur", "controls", &blurs, 30);
	Mat img_1 = imread("iii.PNG", IMREAD_GRAYSCALE);
	cout << "image loaded" <<img_1.cols<< endl;
	Mat img_1p;
	//blur(img_1, img_1p, Size(3, 3));
	//resize(img_1, img_1, Size(), 0.5, 0.5,INTER_AREA);
	VideoCapture cap(1);
	
	//cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	//cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	cout << "camera ok" << endl;
	Mat img_2;
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	cap.read(img_2);
	//= imread("img2.jpg", IMREAD_GRAYSCALE);
	if (!img_1.data || !img_2.data)
	{
		std::cout << " --(!) Error reading images " << std::endl; return -1;
	}
	//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
	int minHessian = 3000;
	//Ptr<SURF> detector = SURF::create();
	Ptr<ORB> detector = ORB::create();
	//detector->setUpright(true);
	//detector->setHessianThreshold(minHessian);
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	Mat descriptors_1, descriptors_2;
	int q = 0;
	while (q < 30) {
		cap.read(img_2);
		imshow("okno", img_2);
		waitKey(20);
		q++;
	}
	while (true) {
		//detector->setHessianThreshold(thresh);
		//GaussianBlur(img_1, img_1p, Size(blurs * 2 + 1, blurs * 2 + 1), 0, 0);
		detector->detectAndCompute(img_1, Mat(), keypoints_1, descriptors_1);
		cap.grab();
		cap.retrieve(img_2);
		//resize(img_2, img_2, Size(), 0.5, 0.5, INTER_AREA);
		cvtColor(img_2, img_2, COLOR_BGR2GRAY);
		detector->detectAndCompute(img_2, Mat(), keypoints_2, descriptors_2);
		//-- Step 2: Matching descriptor vectors using FLANN matcher
		//FlannBasedMatcher matcher;
		//Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
		BFMatcher matcher;
		std::vector< DMatch > matches;
		descriptors_1.convertTo(descriptors_1, CV_32F);
		descriptors_2.convertTo(descriptors_2, CV_32F);
		cout << descriptors_1.cols << " " << descriptors_1.rows << endl;
		cout << descriptors_2.cols << " " << descriptors_2.rows << endl;
		//matcher.match(c);
		cout << "matching" << endl;
		matcher.match(descriptors_1, descriptors_2, matches);
		cout << "matched:" << matches.size() << endl;
		double max_dist = 0; double min_dist = (double)slider_dist / 100.0;
		//-- Quick calculation of max and min distances between keypoints
		for (int i = 0; i < descriptors_1.rows; i++)
		{
			double dist = matches[i].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}
		printf("-- Max dist : %f \n", max_dist);
		printf("-- Min dist : %f \n", min_dist);
		//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
		//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
		//-- small)
		//-- PS.- radiusMatch can also be used here.
		std::vector< DMatch > good_matches;
		for (int i = 0; i < descriptors_1.rows; i++)
		{
			if (matches[i].distance <= max(2 * min_dist, 0.02))
			{
				good_matches.push_back(matches[i]);
			}
		}
		//-- Draw only "good" matches
		Mat img_matches;
		Point center;
		int sum_x = 0;
		int sum_y = 0;
		vector<Point2i> points;
		if ((int)good_matches.size() > 5) {
			int size = (int)good_matches.size();
			for (int i = 0; i < size; i++) {
				int j = good_matches[i].trainIdx;
				sum_x += keypoints_2[j].pt.x;
				sum_y += keypoints_2[j].pt.y;
			}
			center = Point(sum_x / size, sum_y / size);
			circle(img_2, center, 30, Scalar(255, 0, 100), 2, 8, 0);
			cout << "x " << center.x << "y " << center.y << endl;
		}
		drawMatches(img_1p, keypoints_1, img_2, keypoints_2,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		//-- Show detected matches
		imshow("Good Matches", img_matches);
		for (int i = 0; i < (int)good_matches.size(); i++)
		{
			printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
		}
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
