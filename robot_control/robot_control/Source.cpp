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
		if (i > 402)
		{
			
			cmd[0] = 'x';
			stop();
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
				fwd();
				break;
			case 'd':
				printf("skrecam");
				stop();
				break;

			}
			fwd();
			break;

		case 'x':
			stop();
			break;

		}
		printf("i= %d command= %c %c \n", i, cmd[0], cmd[1]);
		printf("dystans: %d\n", dst);
		cmd[1] = (dst > 40) ? 'w' : 'd';
	}

	getchar();
	return 0;
}