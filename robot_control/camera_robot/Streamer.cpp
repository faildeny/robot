#include "Streamer.h"

Streamer::Streamer(int n) {

	sprintf(access_key, "bnjWgY0ZqoURi4TdkV0MLLJvjHSnvQX3");
	//sprintf(access_key, "IX9gCj0xgnLFiimu9cKkTzAP2Ce4Md9F");
	sprintf(bucket_key, "position2");
	sprintf(bucket_name, "position2");
	create_bucket(access_key, bucket_key, bucket_name);
}

void Streamer::send(double posx,double posy) {
		char json[1024];
		//int a = a * 3;
		//posx += a*0.000003 + 0.000001;
		//posy += 0.000001;
		sprintf(json, "{\"key\":\"robot position map\", \"value\":\"%f,%f\"}", posx, posy);
		//sprintf(json, "{\"key\":\"voltage\", \"value\":\"%.2f V\"}",voltage);
		//sprintf(json, "[{\"key\":\"kot\",\"value\":\"czarny\"}]");
		printf("Sending: %s \n", json);
		stream_event(access_key, bucket_key, json);
	}