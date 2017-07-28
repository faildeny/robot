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
public:
	void move();
	void decide(char key, double direction, double distance, int turn);
	int turn();
	int forward();
	void square();
	void headTo(double direction);
	void showStatus();
	RobotControl(int speed_value);
	~RobotControl();
};

