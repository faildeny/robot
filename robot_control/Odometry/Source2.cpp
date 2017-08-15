

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status) {

	//this function automatically gets rid of points for which tracking fails

	vector<float> err;
	Size winSize = Size(21, 21);
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

	calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for (int i = 0; i<status.size(); i++)
	{
		Point2f pt = points2.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x<0) || (pt.y<0)) {
			if ((pt.x<0) || (pt.y<0)) {
				status.at(i) = 0;
			}
			points1.erase(points1.begin() + (i - indexCorrection));
			points2.erase(points2.begin() + (i - indexCorrection));
			indexCorrection++;
		}
		cout<<i<<endl;
	}
	cout << "done feature tracking" << endl;

}

void featureDetection(Mat img_1, vector<Point2f>& points1) {   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	int fast_threshold = 25;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal) {

	string line;
	int i = 0;
	ifstream myfile("/home/avisingh/Datasets/KITTI_VO/00.txt");
	double x = 0, y = 0, z = 0;
	double x_prev, y_prev, z_prev;
	if (myfile.is_open())
	{
		while ((getline(myfile, line)) && (i <= frame_id))
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
			//cout << line << '\n';
			for (int j = 0; j<12; j++) {
				in >> z;
				if (j == 7) y = z;
				if (j == 3)  x = z;
			}

			i++;
		}
		myfile.close();
	}

	else {
		cout << "Unable to open file";
		return 0;
	}

	return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));

}

Mat img_1, img_2;
Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

VideoCapture cap(0);
double scale = 2;

char text[100];
int fontFace = FONT_HERSHEY_PLAIN;
double fontScale = 1;
int thickness = 1;
Point textOrg(10, 50);

//read the first two frames from the dataset
Mat img_1_c;
Mat img_2_c;
vector<Point2f> points1, points2;
vector<uchar> status;
double focal = 1080.375*0.25;
//cv::Point2d pp(644.09296, 348.5013);
Point2d pp(644.09296*0.25, 348.5013*0.25);
Mat E, R, t, mask;
Mat prevImage;
Mat currImage;
vector<Point2f> prevFeatures;
vector<Point2f> currFeatures;
Mat traj = Mat::zeros(600, 600, CV_8UC3);

void initOdometry() {

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 180);
	cap.grab();
	cap.retrieve(img_1_c);
	cap.grab();
	cap.retrieve(img_2_c);

	// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

	// feature detection, tracking
	featureDetection(img_1, points1);        //detect features in img_1
	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2
						
	prevFeatures = points2;
	prevImage = img_2;
	//recovering the pose and the essential matrix
	
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points2, points1, R, t, focal, pp, mask);

	R_f = R.clone();
	t_f = t.clone();

	namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
	namedWindow("Trajectory", WINDOW_AUTOSIZE);// Create a window for display.
}

void update() {
	Mat currImage_c;
	cap.grab();
	cap.retrieve(currImage_c);
	cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
	vector<uchar> status;
	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 10.0, mask);
	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

	Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);


	for (int i = 0; i < prevFeatures.size(); i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
		prevPts.at<double>(0, i) = prevFeatures.at(i).x;
		prevPts.at<double>(1, i) = prevFeatures.at(i).y;

		currPts.at<double>(0, i) = currFeatures.at(i).x;
		currPts.at<double>(1, i) = currFeatures.at(i).y;

		line(currImage_c, prevFeatures.at(i), currFeatures.at(i), Scalar(255, 10, 255), 1);
	}

	//scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));
	//cout << "Scale is " << scale << endl;

	if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

		t_f = t_f + scale*(R_f*t);
		R_f = R*R_f;

	}

	else {
		//cout << "scale below 0.1, or incorrect translation" << endl;
	}

	// lines for printing results
	// myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

	// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
	if (prevFeatures.size() < MIN_NUM_FEAT) {

		featureDetection(prevImage, prevFeatures);
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
	}

	prevImage = currImage.clone();
	prevFeatures = currFeatures;

	int x = int(t_f.at<double>(0)) + 300;
	int y = int(t_f.at<double>(2)) + 100;
	circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

	rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
	sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
	putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

	imshow("Road facing camera", currImage_c);
	imshow("Trajectory", traj);

}