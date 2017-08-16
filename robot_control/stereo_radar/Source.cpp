#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

using namespace std;
using namespace cv;


void klik(int event, int wx, int wy, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		Point* punktKlik = (Point*)userdata;
		punktKlik->x = wx;
		punktKlik->y = wy;
	}
}

void map3d(Mat &map, Mat image3d) {
	int center_x = 600;
	int center_y = 500;
	int j = 20;
	map = Mat::zeros(600, 1280, CV_8UC3);
	Rect robot_rect(center_x - 10, center_y - 10, 20, 30);
	rectangle(map, robot_rect, Scalar(30, 255, 60), 2);
	for (int j = 20; j < 21; j++) {
		for (int i = 0; i < image3d.cols; i++) {

			int x = int(5 * image3d.at<Vec3f>(j, i)[0]) + center_x;
			int y = int(-5 * image3d.at<Vec3f>(j, i)[2]) + center_y;
			circle(map, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
		}
	}
}



int main(void)
{
	//Mat img1 = imread("s1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//Mat img2 = imread("s2.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	VideoCapture cap1(1);
	VideoCapture cap2(0);

	Size frameSize(1280, 720);

	cap1.set(CAP_PROP_FRAME_WIDTH, frameSize.width);
	cap1.set(CAP_PROP_FRAME_HEIGHT, frameSize.height);
	cap2.set(CAP_PROP_FRAME_WIDTH, frameSize.width);
	cap2.set(CAP_PROP_FRAME_HEIGHT, frameSize.height);

	Size originalframeSize(1280, 720);

	Mat temp1;
	Mat temp2;
	Mat frame(frameSize, CV_8U);
	Mat frame3(320, 180, CV_8U);
	Mat frame2(frame.size(), frame.type());

	cap1.grab();
	cap1.retrieve(temp1);

	Size tempSize = temp1.size();
	Rect area(0, tempSize.height - frameSize.height, frameSize.width, frameSize.height);
	double scaleRatio = (double)frameSize.width / (double)originalframeSize.width;
	Point2i punkt(50, 50);

	//StereoBM::StereoMatcher;
	namedWindow("okno");
	resizeWindow("okno", 500, 1000);
	int numdis = 7; //7
	int wsize = 9; //6
	int prefilter = 31;
	int texturet = 10;
	int speckleSize = 100;
	int unique = 15;
	int dispmax = 1;
	int roiw1 = 100;
	int roiw2 = 200;
	int ratio = 90; //90
	int offset = 125; //125
	int exposure = 5;
	createTrackbar("Num Disparities", "okno", &numdis, 16);
	createTrackbar("Window Size", "okno", &wsize, 40);
	createTrackbar("Prefilter", "okno", &prefilter, 100);
	createTrackbar("Texture threshold", "okno", &texturet, 50);
	createTrackbar("Speckle Window", "okno", &speckleSize, 500);
	createTrackbar("Uniqueness ratio", "okno", &unique, 100);
	createTrackbar("MinDisp", "okno", &dispmax, 10);
	createTrackbar("ROI 1", "okno", &roiw1, 4000);
	createTrackbar("ROI 2", "okno", &roiw2, 500);
	createTrackbar("Scale", "okno", &ratio, 500);
	createTrackbar("Offset", "okno", &offset, 400);

	Mat disp = Mat::zeros(frame3.size(), CV_32FC1);
	Mat disp2(frame3.size(), CV_32FC1);
	disp = disp2;
	Mat disp_scan = disp;
	Mat disp8;

	Ptr<StereoBM> bm = StereoBM::create(16, 9);

	int SADWindowSize = 15;
	int numberOfDisparities = 32;
	Rect roi1(1, 1, 100, 100);
	Rect roi2(1, 1, roiw2, roiw2);

	////////////////////////////////////
	Mat CM1 = Mat(3, 3, CV_64FC1);
	Mat CM2 = Mat(3, 3, CV_64FC1);
	Mat D1, D2;
	Mat R, T, E, F;
	//Mat img1, img2;
	Mat R1, R2, P1, P2, Q;
	Mat map1x, map1y, map2x, map2y;
	Mat imgU1, imgU2;

	FileStorage fs1("extrinsics.yml", FileStorage::READ);
	if (!fs1.isOpened())
	{
		cout << "Failed to open " << endl;
		return 1;
	}

	fs1["M1"] >> CM1;
	fs1["M2"] >> CM2;
	fs1["D1"] >> D1;
	fs1["D2"] >> D2;
	fs1["R"] >> R;
	fs1["T"] >> T;
	fs1["E"] >> E;
	fs1["F"] >> F;
	fs1["R1"] >> R1;
	fs1["R2"] >> R2;
	fs1["P1"] >> P1;
	fs1["P2"] >> P2;
	fs1["Q"] >> Q;

	printf("Done Calibration\n");
	printf("D1: %d", D1.cols);
	printf("Starting Rectification\n");



	initUndistortRectifyMap(CM1, D1, R1, P1, frame.size(), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(CM2, D2, R2, P2, frame2.size(), CV_32FC1, map2x, map2y);


	namedWindow("Trajectory", WINDOW_NORMAL);// Create a window for display.
	Mat cropped1, cropped2;
	Range crop_width_range(0, frame.cols);
	Range crop_height_range(frame.rows*0.5 - 40, frame.rows*0.5);
	Mat image3d;
	Mat traj;

	while (true) {

		cap1.grab();
		cap2.grab();
		cap1.grab();
		cap2.grab();
		cap1.grab();
		cap2.grab();
		cap1.grab();
		cap2.grab();
		cap1.grab();
		cap2.grab();
		cap1.retrieve(temp1);
		cap2.retrieve(temp2);

		frame = temp1(area);
		frame2 = temp2(area);

		cvtColor(frame, frame, COLOR_BGR2GRAY);
		cvtColor(frame2, frame2, COLOR_BGR2GRAY);

		remap(frame, frame, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		remap(frame2, frame2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		cropped1 = frame(crop_height_range, crop_width_range);
		cropped2 = frame2(crop_height_range, crop_width_range);

		//resize(frame, frame, Size(320, 180));
		//resize(frame2, frame2, Size(320, 180));

		//imshow("lewa", temp1);
		//imshow("prawa", temp1 - temp2);
		numberOfDisparities = numdis * 16;
		SADWindowSize = wsize * 2 + 1;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setPreFilterCap(prefilter); //31
		bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
		bm->setMinDisparity(0);
		bm->setNumDisparities(numberOfDisparities);
		bm->setTextureThreshold(texturet); //10
		bm->setUniquenessRatio(unique); //15
		bm->setSpeckleWindowSize(speckleSize); //200
		bm->setSpeckleRange(32); //32
		bm->setDisp12MaxDiff(12); //1


								  //bm->compute(frame, frame2, disp);
		bm->compute(cropped1, cropped2, disp_scan);

		//resize(disp8, disp8, Size(), 3, 3, INTER_LINEAR);
		Mat preview;
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
		disp8.convertTo(preview, -1, double(ratio) / 50., offset - 200);

		//Adding the color preview
		applyColorMap(preview, preview, COLORMAP_JET);

		setMouseCallback("disparity", klik, (void*)&punkt);
		//Scalar odstep = disp8.at<uchar>(punkt);
		//const double a = 110;//260.357;
		//const double b = 0.0;
		//double dystans = a / (odstep.val[0]) + b;
		//z_left = b * f / d.
		disp.convertTo(disp, CV_32FC1);
		//float d = disp.at<float>(punkt);
		//cout << "T value: " << T.at<double>(0, 0) << " CM1 value: " << CM1.at<double>(0, 0) << endl;
		//double dystans = -T.at<double>(0,0)*CM1.at<double>(0,0)/d*16.f;
		//double dystans = -0.001 / Q.at<double>(3, 2)*Q.at<double>(2, 3) / d*16.f;

		//String odstepText = to_string(dystans);
		//odstepText = odstepText.substr(0, 4);
		//odstepText += " m";
		//cout << "wartosc odleglosci: " << odstep.val[0] << endl;

		//putText(preview, odstepText, punkt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 250, 255), 2, CV_AA, 0);


		//scanning line part
		disp_scan.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
		disp8.convertTo(preview, -1, double(ratio) / 50., offset - 200);
		applyColorMap(preview, preview, COLORMAP_JET);


		disp_scan.convertTo(disp_scan, CV_32FC1);
		reprojectImageTo3D(disp_scan, image3d, Q);
		map3d(traj, image3d);

		imshow("Trajectory", traj);

		// Displaying the disparity map
		imshow("disparity", preview);

		int iKey = waitKey(50);
		if (iKey == 27)
		{
			break;
		}
	}
	return 0;
}