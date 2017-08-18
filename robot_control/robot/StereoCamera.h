#pragma once
#include "Camera.h"
class StereoCamera {
private:
	Ptr<StereoBM> bm;
public:
	int SADWindowSize = 15;
	int numberOfDisparities = 32;
	int numdis = 7;
	int wsize = 9;
	int prefilter = 31;
	int texturet = 10;
	int speckleSize = 100;
	int unique = 8;
	int dispmax = 1;
	int roiw1 = 100;
	int roiw2 = 200;
	int ratio = 90; 
	int offset = 125;
	int exposure = 5;
public:
	StereoCamera();
	~StereoCamera();
	void showMenu();
	void setParams();
	bool setExtrinsics();
	void match(Mat frame1, Mat frame2, Mat &disp);
};