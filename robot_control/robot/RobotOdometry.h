#pragma once

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;


class RobotOdometry {
public:
	Point2d position;
	double azimuth;
	double angleDeg;
	double move_step;
	Mat map;
	Mat background;
	Mat robot_shape;
	int enc_left;
	int enc_right;
	int enc_l_dir;
	int enc_r_dir;

	double posx;
	double posy;

private:
	Size map_size;
	
	int enc_left_old;
	int enc_right_old;
	int enc_diff_left;
	int enc_diff_right;
	int left;
	int right;
	double angle_step;
	int view_dist;
	int view_angleDeg;
	double view_angle;
	int view_res;
	int center_x;
	int center_y;

	double posx_base;
	double posy_base;

	Mat image3d;
	Mat scan_line1, scan_line2;

public:
	RobotOdometry();

	~RobotOdometry();

	void decodeEncoders();

	void updateCoordinates();

	void updateMap();

	void markTarget();
	
	void map3d(Mat &map, Mat image3d);

private:
	void drawRobot(Mat& image, Point centerPoint, Size rectangleSize, double rotationDegrees);

	void drawCurrentArea(Mat& background, Point center, double azimuth);

};