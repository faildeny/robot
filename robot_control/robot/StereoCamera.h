#pragma once
#include "Camera.h"
class StereoCamera {
private:
	Ptr<StereoBM> bm;
public:
	int SADWindowSize = 15;
	int numberOfDisparities = 32;
	int numdis = 3;
	int wsize = 9;
	int prefilter = 31;
	int texturet = 10;
	int speckleSize = 100;
	int unique = 15;
	int dispmax = 1;
	int roiw1 = 100;
	int roiw2 = 200;
	int ratio = 90; 
	int offset = 125;
	int exposure = 200;
	int focalcenter = 10;
	int focallength = 10;
	int baseline = 10;

	Mat R, T, E, F, Q,Qs;

public:
	StereoCamera();
	~StereoCamera();
	void showMenu();
	void setParams();
	bool setExtrinsics(String filename,float scale);
	void calculateQs(float scale);
	void match(Mat frame1, Mat frame2, Mat &disp);
};