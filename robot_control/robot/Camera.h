#pragma once
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

using namespace std;
using namespace cv;

class Camera:public VideoCapture {
public:
	int width;
	int height;
	int exposure;
	struct CameraParameters {
		Mat CM = Mat(3, 3, CV_64FC1);
		Mat D;
		Mat RC, P;
	};
	CameraParameters params;
	Mat mapx, mapy;

public:
	Camera(int n);
	~Camera();
	void setSize(int width, int height);
	void setExp(int e);
	bool setIntrinsics(String filename, int n);
	void scaleIntrinsics(double scale);
	void setUndistortRectifyMap(Size frame_size);
	void remapFrame(Mat& frame);
};
