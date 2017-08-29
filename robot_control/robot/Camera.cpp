#include "Camera.h"


Camera::Camera(int n)
:VideoCapture(n),
width(1280),
height(720)
{
	if (open(n)) {
		setSize(width, height);
	}
	else
	{
		cout << "Can't initialize camera: " << n << endl;
	}
};

Camera::~Camera() {};

void Camera::setSize(int width, int height) {
	set(CAP_PROP_FRAME_WIDTH, width);
	set(CAP_PROP_FRAME_HEIGHT, height);
};

void Camera::setExp(int e)
{
	exposure = e;
	set(CAP_PROP_EXPOSURE, exposure);
	
};

bool Camera::setIntrinsics(String filename, int n) {
	FileStorage fs1(filename, FileStorage::READ);
	if (!fs1.isOpened())
	{
		cout << "Failed to open " << endl;
		return 0;
	}
	if (n == 1) {
		fs1["M1"] >> params.CM;
		fs1["D1"] >> params.D;
		fs1["R1"] >> params.RC;
		fs1["P1"] >> params.P;
	}
	else {
		fs1["M2"] >> params.CM;
		fs1["D2"] >> params.D;
		fs1["R2"] >> params.RC;
		fs1["P2"] >> params.P;
	}
	return 1;
}

void Camera::setUndistortRectifyMap(Size frame_size) {
	initUndistortRectifyMap(params.CM, params.D, params.RC, params.P, frame_size, CV_32FC1, mapx, mapy);
};

void Camera::remapFrame(Mat& frame) {
	remap(frame, frame, mapx, mapy, INTER_LINEAR, BORDER_CONSTANT, Scalar());
}

void Camera::scaleIntrinsics(double scale) {
	params.CM.at<double>(0, 0) *= scale;
	params.CM.at<double>(0, 2) *= scale;
	params.CM.at<double>(1, 1) *= scale;
	params.CM.at<double>(1, 2) *= scale;

	params.D = params.D*scale*0;

	params.P.at<double>(0, 0) *= scale;
	params.P.at<double>(0, 2) *= scale;
	params.P.at<double>(1, 1) *= scale;
	params.P.at<double>(1, 2) *= scale;

}