#pragma once
#include <iostream>
#include <stdio.h>
extern "C" {
#include "gopigo.h"
}

class RobotControl
{
public:
	char cmd[3];
	int i;
	int step;
	double dist;
	double dir;
	bool busy;
	bool found;
	struct RobotSpeed {
		int rotate;
		int forward;
	};
	RobotSpeed speed;
	int line_dist;
	int enc_left;
	int enc_right;
	int enc_begin_left;
	int enc_begin_right;

	
public:
	void move();
	void decide(char key, double direction, double distance, int turn, double target_size);
	int turn();
	int forward();
	void square();
	void headTo(double direction);
	void showStatus();
	void readEncoders(int &left,int &right, int &l_dir, int &r_dir);

	RobotControl(int speed_value);
	~RobotControl();
};

