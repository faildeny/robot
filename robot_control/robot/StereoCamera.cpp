#include "StereoCamera.h"

StereoCamera::StereoCamera() {

bm= StereoBM::create(16, 9);
setParams();
};

StereoCamera::~StereoCamera() {};

void StereoCamera::showMenu() {
	namedWindow("Camera and stereo settings");
	
	createTrackbar("Number of disparities", "Camera and stereo settings", &numdis, 16);
	createTrackbar("Window size", "Camera and stereo settings", &wsize, 40);
	createTrackbar("Uniqueness ratio", "Camera and stereo settings", &unique, 100);
	createTrackbar("Scale", "Camera and stereo settings", &ratio, 500);
	createTrackbar("Offset", "Camera and stereo settings", &offset, 400);
	createTrackbar("Exposure", "Camera and stereo settings", &exposure, 15);
};

void StereoCamera::setParams() {
	numberOfDisparities = numdis * 16;
	SADWindowSize = wsize * 2 + 1;
	bm->setPreFilterCap(prefilter);
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(texturet);
	bm->setUniquenessRatio(unique);
	bm->setSpeckleWindowSize(speckleSize);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(12);
};

bool StereoCamera::setExtrinsics(String filename,float scale) {

		FileStorage fs1(filename, FileStorage::READ);
		if (!fs1.isOpened())
		{
			cout << "Failed to open " << endl;
			return 0;
		}
		fs1["R"] >> R;
		fs1["T"] >> T;
		fs1["E"] >> E;
		fs1["F"] >> F;
		fs1["Q"] >> Q;
		calculateQs(scale);
		return 1;
};

void StereoCamera::match(Mat frame1, Mat frame2, Mat &disp) {
	bm->compute(frame1,frame2, disp);
};


void StereoCamera::calculateQs(float scale) {
	Qs = Q.clone();
	Qs.at<double>(0, 0) = 1.0;
	Qs.at<double>(0, 1) = 0.0;
	Qs.at<double>(0, 2) = 0.0;
	Qs.at<double>(0, 3) *= scale; //cx
	Qs.at<double>(1, 0) = 0.0;
	Qs.at<double>(1, 1) = 1.0;
	Qs.at<double>(1, 2) = 0.0;
	Qs.at<double>(1, 3) *= scale;  //cy
	Qs.at<double>(2, 0) = 0.0;
	Qs.at<double>(2, 1) = 0.0;
	Qs.at<double>(2, 2) = 0.0;
	Qs.at<double>(2, 3) *= 1;  //Focal
	Qs.at<double>(3, 0) = 0.0;
	Qs.at<double>(3, 1) = 0.0;
	Qs.at<double>(3, 2) *= 1;    //1.0/BaseLine
	Qs.at<double>(3, 3) = 0.0;
	Qs.convertTo(Qs, CV_32F);
};