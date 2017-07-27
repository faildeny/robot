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
	int status;
	double dist;
	bool busy;
public:
	void move();
	void decide(char key, int direction, double distance, int turn);
	int turn();
	int forward();
	void square();
	RobotControl();
	~RobotControl();
};

