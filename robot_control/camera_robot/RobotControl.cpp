#include "RobotControl.h"



RobotControl::RobotControl()
{
	if (init() == -1) {
		exit(1);
	}

	cmd[0] = 'w';
	cmd[1] = 'x';
	cmd[2] = 'd';
	set_speed(100);
	enc_tgt(1, 1, 10);
	i = 0;
	status = 0;
	dist = 0;
	busy = false;
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

		printf("mozna jechac: ");
		switch (cmd[1])
		{
		case 'w':
			switch (cmd[2])
			{
			case 'd':
				printf("skrecam w prawo");
				led_on(0);
				led_off(1);
				set_speed(40);
				right_rot();
				break;
			case 'a':
				printf("skrecam w lewo");
				led_on(1);
				led_off(0);
				set_speed(40);
				left_rot();
				break;
			case 'w':
				printf("jade");
				led_off(0);
				led_off(1);
				//motor1(1, 30);
				//motor2
				set_speed(60);
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
				set_speed(40);
				right_rot();
				break;
			case 'a':
				printf("skrecam w lewo");
				led_on(0);
				led_on(1);
				set_speed(40);
				left_rot();
				break;
			case 'w':
				set_speed(40);
				right_rot();
				break;
			}
			break;
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

void RobotControl::decide(char key, int direction, double distance, int turn) {

	i++;
	cmd[0] = key;
	dist= distance;

	cmd[1] = (distance > 0.3 && distance < 10) ? 'w' : 'r';
	//cmd[1] = (read_enc_status() != 1) ? 'w' : 'r';
	switch (cmd[1])
	{
	case 'w':

		if (direction < 10 && direction > -10) {
			cmd[2] = 'w';
		}
		else
		{
			cmd[2] = (direction < -10) ? 'a' : 'd';
		}
		break;

	case 'r':
		cmd[2] = (turn < 0) ? 'a' : 'd';
		printf("roznica wyboru kierunku: %d", turn);
		break;
	}
	//print current command and distance

	printf(" i= %d command= %c %c %c \n", i, cmd[0], cmd[1], cmd[2]);
	printf("distance: %f direction: %d \n", dist,direction);
	
	// encoders
	//printf("enc_read(0): %d enc_read(1): %d \n", enc_read(0), enc_read(1));
	//printf("read_enc_status(): %d \n", read_enc_status());
}

int RobotControl::turn() {
	if (!busy) {
		enc_tgt(1, 1, 20);
		cmd[2] = 'd';
		busy = true;
	}

	if (!read_enc_status())
	{
		busy = false;
		return 1;
	}
	return 0;
}

int RobotControl::forward() {
	if (!busy) {
		enc_tgt(1, 1, 40);
		cmd[2] = 'w';
		busy = true;
	}

	if (!read_enc_status())
	{
		busy = false;
		return 1;
	}
	return 0;
}

void RobotControl::square() {

	switch (status) {
		case 0:
			if (forward()) status = 1;
			break;

		case 1:
			if (turn()) status= 2;
			break;

		case 2:
			if (forward()) status = 3;
			break;

		case 3:
			if (turn()) status = 4;
			break;

		case 4:
			printf("Order has been completed! \n");
			break;
	}
}