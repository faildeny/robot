#include <stdio.h>
extern "C" {
#include "gopigo.h"
}


using namespace std;

int main(void) {

	if (init() == -1) {
		exit(1);
	}


	int komenda = 0;
	int i = 0;
	int dst = 0;

	while (1) {
		i++;
		if (i > 500)
		{
			komenda = 0;
		}

		switch (komenda)
		{
		case 1:
			printf("jade!");
			fwd();
			break;

		case 0:
			stop();
			break;

		}
		printf("i= %d komenda= %d \n", i, komenda);
		printf("dystans: %d\n", dst);
		komenda = (dst > 40) ? 1 : 0;
	}

	getchar();
	return 0;
}