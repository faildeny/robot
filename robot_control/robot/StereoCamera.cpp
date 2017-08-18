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

bool StereoCamera::setExtrinsics() {
	return 0;
};

void StereoCamera::match(Mat frame1, Mat frame2,Mat &disp) {
	bm->compute(frame1,frame2, disp);
};