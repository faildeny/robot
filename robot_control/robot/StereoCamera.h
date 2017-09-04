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
	Mat preview;
	Mat disp;
	Mat disp8;
	//Default point for distance measurement
	Point2i punkt;

private:
	double farthest_dist, nearest_dist;
	ostringstream ss;
	Size disp_size;
	Rect area_rect;
	Range area_h;
	Range area_w;
	double centdistance;
	String text;

	int sum_l, sum_r;
	int turn;
	int border;
	Range dir_area_l;
	Range dir_area_r;
	Range dir_area_h;
	Scalar sum_l_scalar;
	Scalar sum_r_scalar;

	Rect left_area;
	Rect right_area;

public:
	StereoCamera();
	~StereoCamera();
	void showMenu();
	void setParams();
	bool setExtrinsics(String filename,float scale);
	void calculateQs(float scale);
	void match(Mat frame1, Mat frame2);
	void preparePreview();
	void drawDashboard();
	double distCentralArea();
	double avoidDirection();
};