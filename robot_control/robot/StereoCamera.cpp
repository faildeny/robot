#include "StereoCamera.h"

StereoCamera::StereoCamera() {

bm= StereoBM::create(16, 9);
setParams();
};

StereoCamera::~StereoCamera() {};

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