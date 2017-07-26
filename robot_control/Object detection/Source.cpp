#include "opencv2\objdetect\objdetect.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

void detectAndDisplay(Mat frame);

String face_cascade_name = "cascade.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Rozpoznanie twarzy";
RNG rng(12345);

int main(int argc, const char** argv)
{
	VideoCapture capture(1);
	Mat frame;

	if (!face_cascade.load(face_cascade_name)) { printf("classifier cannot be loaded \n"); return -1; }
	if (!eyes_cascade.load(eyes_cascade_name)) { printf("classifier eyes cannon be loaded \n"); return -1; }

	if (capture.isOpened())
	{
		while (true)
		{
			capture.grab();
			capture.retrieve(frame);

			if(!frame.empty())
			{
				detectAndDisplay(frame);
				//imshow("okno", frame);
				//int iKey = waitKey(5);
			}
			else
			{
				printf("no frame from the camera \n");
				break;
			}
			int c = waitKey(10);
			if ((char)c == 'c') { 
				break; 
			}
		}
	}
	return 0;
}

void detectAndDisplay(Mat frame)
{
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0, Size(30, 30));
	for (int i = 0; i < faces.size(); i++)
	{
		Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
		Rect rectFace(faces[i].x, faces[i].y, faces[i].width, faces[i].height);
		rectangle(frame, faces[i], Scalar(0, 255, 200), 2, 8);
		
		Mat faceROI = frame_gray(faces[i]);
		std::vector<Rect> eyes;
		//eyes_cascade.detectMultiScale(frame, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

		for (int j = 0; j < eyes.size(); j++)
		{
			rectangle(frame, eyes[j], Scalar(255, 0, 200), 2, 8);
		}
	}
	imshow("oknno", frame);
}