#include "RobotControl.h"



RobotControl::RobotControl(int speed_value)
{
	if (init() == -1) {
		exit(1);
	}

	cmd[0] = 'w';
	cmd[1] = 'x';
	cmd[2] = 'd';
	speed.forward = speed_value;
	speed.rotate = speed_value-10;
	set_speed(speed.forward);
	//enc_tgt(1, 1, 10);
	enc_left = 0;
	enc_right = 0;
	i = 0;
	step = 0;
	dist = 0;
	dir = 0;
	busy = false;
	found = false;
}

RobotControl::~RobotControl()
{
}

void RobotControl::move() {
	if (i > 1000)
	{
		cmd[0] = 'x';
		stop();
		led_off(0);
		led_off(1);
		exit(0);
	}

	switch (cmd[0])
	{
	case 'w':

		//printf("mozna jechac: ");
		switch (cmd[1])
		{
		case 'w':
			switch (cmd[2])
			{
			case 'd':
				printf("skrecam w prawo");
				led_on(0);
				led_off(1);
				set_speed(speed.rotate);
				right_rot();
				break;
			case 'a':
				printf("skrecam w lewo");
				led_on(1);
				led_off(0);
				set_speed(speed.rotate);
				left_rot();
				break;
			case 'w':
				//printf("jade");
				led_off(0);
				led_off(1);
				set_speed(speed.forward);
				fwd();
				break;
			}
			break;

		case 'r':
			printf("za blisko ");
			switch (cmd[2])
			{
			case 'd':
				printf("skrecam w prawo");
				led_on(1);
				led_on(0);
				set_speed(speed.rotate);
				right_rot();
				break;
			case 'a':
				printf("skrecam w lewo");
				led_on(0);
				led_on(1);
				set_speed(speed.rotate);
				left_rot();
				break;
			case 'w':
				set_speed(speed.rotate);
				right_rot();
				break;
			}
			break;
		case 'x':
			printf("I FOUND IT \n");
			stop();
		}

		break;

	case 'x':
		stop();
		break;
	case 'c':
		stop();
		led_off(0);
		led_off(1);
		exit(0);
		break;
	case 'i':
		set_speed(80);
		fwd();
		break;
	case 'j':
		set_speed(80);
		left_rot();
		break;
	case 'l':
		set_speed(80);
		right_rot();
		break;
	case 'k':
		set_speed(60);
		bwd();
		break;

	}

}

void RobotControl::decide(char key, double direction, double distance, int turn, double target_size) {

	i++;
	cmd[0] = key;
	dist= distance;
	dir = direction;

	cmd[1] = (distance > 0.35 && distance < 10) ? 'w' : 'r';
	if (target_size > 0.3) cmd[1] = 'x';
	//cmd[1] = (read_enc_status() != 1) ? 'w' : 'r';
	
	switch (cmd[1])
	{
	case 'w':

		if (direction < 0.07 && direction > -0.07) {
			cmd[2] = 'w';
		}
		else
		{
			cmd[2] = (direction < -0.07) ? 'q' : 'e';
			headTo(direction);
		}
		break;

	case 'r':
		cmd[2] = (turn < 0) ? 'a' : 'd';
		printf("roznica wyboru kierunku: %d \n", turn);
		break;
	}
	//print current command and distance

	
	
	// encoders
	//printf("enc_read(0): %d enc_read(1): %d \n", enc_read(0), enc_read(1));
	//printf("read_enc_status(): %d \n", read_enc_status());
}

int RobotControl::turn() {
	if (!busy) {
		enc_begin_left = enc_left;
		//enc_tgt(1, 1, 8);
		busy = true;
		
	}

	cmd[2] = 'd';
	printf("skrecac w prawo \n");

	if (enc_left-enc_begin_left>7)
	{
		busy = false;
		return 1;
		printf("skrecilem w prawo \n");

	}
	return 0;
}

int RobotControl::forward() {
	if (!busy) {
		enc_begin_left = enc_left;
		enc_begin_right = enc_right;
		//enc_tgt(1, 1, 40);
		busy = true;
	}

	cmd[2] = 'w';

	if (enc_left - enc_begin_left>40 && enc_right - enc_begin_right>40)
	{
		busy = false;
		return 1;
	}
	return 0;
}

void RobotControl::square() {

	switch (step) {
		case 0:
			if (forward()) step++;
			break;

		case 1:
			if (turn()) step++;
			break;

		case 2:
			if (forward()) step++;
			break;

		case 3:
			if (turn()) step++;
			break;
		
		case 4:
			if (forward()) step++;
			break;

		case 5:
			if (turn()) step++;
			break;

		case 6:
			if (forward()) step++;
			break;

		case 7:
			if (turn()) step++;
			break;

		case 8:
			printf("Order has been completed! \n");
			break;
	}
}

void RobotControl::headTo(double direction) {
	//if (!busy) {
		//enc_tgt(1, 1, 1);
	//	busy = true;
		printf("planowany skok enkodera: 1 moglo byc: %d \n", 1+int(4.0*direction));

	printf("ustawiam sie! \n");
	set_speed(speed.rotate);
	
		switch (cmd[2]) {
		case 'q':
			printf("jade w lewo \n");
			enc_tgt(0, 1, 1);
			//left_rot();
			left();
			break;
		case 'e':
			printf("jade w prawo \n");
			enc_tgt(1, 0, 1);
			//right_rot();
			right();
			break;
		}
		printf("enocder status: %d \n ", read_enc_status());
		while (read_enc_status()) {
		//if (!read_enc_status()) {
		//	busy = false;
		//}
		}
		printf("enocder status: %d \n ", read_enc_status());
		stop();
}

void RobotControl::showStatus() {
	printf(" i= %d previous command= %c %c %c \n", i, cmd[0], cmd[1], cmd[2]);
	printf("distance: %f direction: %f \n \n", dist, dir);
}

void RobotControl::readEncoders(int &left,int &right,int &l_dir, int &r_dir) {
	enc_left= enc_read(1);
	enc_right = enc_read(0);
	left = enc_left;
	right = enc_right;
	l_dir = 1;
	r_dir = 1;
	switch (cmd[0]) {
	case 'w':
		if (cmd[2] == 'a') l_dir = -1;
		if (cmd[2] == 'd') r_dir= -1;
		break;

	case 'j':
		l_dir = -1;
		break;
	case 'l':
		r_dir = -1;
		break;
	}
	
}