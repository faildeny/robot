//#include "VisualOdometry.h"

#include <iostream>
#include <stdio.h>
#include "initial-state.h"

#define _CRT_SECURE_NO_WARNINGS


int main() {
	//VideoCapture cap(0);
	//Mat image;
	//Mat image2;
	//cap.grab();
	//cap.retrieve(image);
	
	
	//VisualOdometry vis_odo(image);

	printf("HEllo");
	char access_key[100];
	char bucket_key[100];
	char bucket_name[100];
	//char access_key='a';
	sprintf_s(access_key, "IX9gCj0xgnLFiimu9cKkTzAP2Ce4Md9F");
	sprintf_s(bucket_key, "2");
	sprintf_s(bucket_name, "nowy");
	//create_bucket(access_key, bucket_key, bucket_name);

	
		


	while (true) {


		//cap.grab();
		//cap.retrieve(image);
		//vis_odo.update(image);
		int i = 8;
		char json[30];
		int a = i * 3;
		sprintf_s(json, "{\"key\":\"position\",\"value\":%d}", i);
		printf("content %s \n koniec ", json);
		//sprintf(json, "{\"data\" : \"%d\", \"inne\" : \"12\"}",i);
		//sprintf(json, "{\"bucketKey\" : \"%s\", \"bucketName\" : \"%s\"}",bucket_key, bucket_name);
		//stream_event(access_key, bucket_key, json);
		printf("Sending data \n");
		getchar();

		//waitKey(100);
	}
	return 0;
}