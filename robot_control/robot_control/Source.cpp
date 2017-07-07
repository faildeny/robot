#include <stdio.h>
extern "C" {
#include "gopigo.h"
}


using namespace std;

int main(void) {

	if (init() == -1) {
		exit(1);
	}

	char cmd[2];
	cmd[0] = 'w';
	cmd[1] = 'x';

	int i = 0;
	int dst = 0;


	while (1) {
		
		i++;
		if (i > 100)
		{
			
			cmd[0] = 'x';
			stop();
			exit(0);
			break;
		}

		switch (cmd[0])
		{
		case 'w':
			
			printf("mozna jechac: ");
			switch (cmd[1])
			{
			case 'w':
				printf("jade");
				led_off(0);
				led_off(1);
				fwd();
				break;
			case 'd':
				printf("skrecam");
				led_on(1);
				led_on(0);
				right_rot();


				break;
			}

			break;

		case 'x':
			stop();
			break;

		}
		dst = us_dist(15);
		cmd[1] = (dst > 40) ? 'w' : 'd';
		printf("i= %d command= %c %c \n", i, cmd[0], cmd[1]);
		printf("dystans: %d\n", dst);
		
	}

	//getchar();
	return 0;
}