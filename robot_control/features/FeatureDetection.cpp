#include "FeatureDetection.h"

FeatureDetection::FeatureDetection(cv::String filename, int dist) {
	namedWindow("controls");
	slider_dist = dist;
	createTrackbar("dist", "controls", &slider_dist, 100000);
	createTrackbar("thresh", "controls", &thresh, 5000);
	img_1 = imread(filename, IMREAD_GRAYSCALE);
	cout << "image loaded" << img_1.cols << endl;

	if (!img_1.data || !img_2.data)
	{
		std::cout << " --(!) Error reading images " << std::endl;
	}
	//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
	int minHessian = 3000;
	//Ptr<SURF> detector = SURF::create();
	detector = ORB::create();
	//detector->setUpright(true);
	//detector->setHessianThreshold(minHessian);
	detector->detectAndCompute(img_1, Mat(), keypoints_1, descriptors_1);
};

FeatureDetection::~FeatureDetection() {};

void FeatureDetection::search(Mat image) {

	area = Rect(0, image.rows*0.0, image.cols, image.rows*1);
	img_2= image(area);
	//resize(img_2, img_2, Size(), 0.5, 0.5, INTER_AREA);
	cvtColor(img_2, img_2, COLOR_BGR2GRAY);
	detector->detectAndCompute(img_2, Mat(), keypoints_2, descriptors_2);
	//-- Step 2: Matching descriptor vectors using FLANN matcher
	//FlannBasedMatcher matcher;
	//Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	std::vector< DMatch > good_matches;
	int a = 2;
	//if (a == 1) {
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
		
		for (int i = 0; i < descriptors_1.rows; i++)
		{
			if (matches[i].distance <= max(2 * min_dist, 0.02))
			{
				good_matches.push_back(matches[i]);
			}
		}
	//}
	////if (a == 2) {
		//std::vector<std::vector<cv::DMatch>> matches;
		//cv::BFMatcher matcher;
		//matcher.knnMatch(descriptors_1, descriptors_2, matches, 2);  // Find two nearest matches
		////vector<cv::DMatch> good_matches;
		//cout << "number of matches: " << matches.size() << endl;
		//for (int i = 0; i < matches.size(); ++i)
		//{
		//	const float ratio = (double)slider_dist / 1000.0; // As in Lowe's paper; can be tuned
		//	cout << matches[i][0].distance << " and " << matches[i][1].distance << endl;
		//	if (matches[i][0].distance < ratio * matches[i][1].distance)
		//	{
		//		cout << "match!" << endl;
		//		good_matches.push_back(matches[i][0]);
		//	}
		//}
	////}
	//-- Draw only "good" matches
	Mat img_matches;
	Point center;
	int sum_x = 0;
	int sum_y = 0;
	vector<Point2i> points;
	cout << "good match size: " << good_matches.size() << endl;
	if ((int)good_matches.size() > 1) {
		int size = (int)good_matches.size();
		for (int i = 0; i < size; i++) {
			int j = good_matches[i].trainIdx;
			sum_x += keypoints_2[j].pt.x;
			sum_y += keypoints_2[j].pt.y;
		}
		center = Point(sum_x / size, sum_y / size);
		circle(img_2, center, 30, Scalar(255, 0, 100), 2, 8, 0);
		//cout << "x " << center.x << "y " << center.y << endl;
	}
	drawMatches(img_1, keypoints_1, img_2, keypoints_2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//-- Show detected matches
	imshow("Good Matches", img_matches);
	for (int i = 0; i < (int)good_matches.size(); i++)
	{
		printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
	}
}