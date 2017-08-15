#include <iostream>
#include <stdio.h>
#include "initial-state.h"

class Streamer {
private:
	char access_key[100];
	char bucket_key[100];
	char bucket_name[100];

public:
	Streamer(int n);
	void send(double posx, double posy);

};