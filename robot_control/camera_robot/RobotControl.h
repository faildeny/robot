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
	double dist;
public:
	void move();
	void decide(char key, int direction, double distance, int turn);
	RobotControl();
	~RobotControl();
};

