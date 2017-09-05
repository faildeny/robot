#include "StereoCamera.h"

StereoCamera::StereoCamera() {

bm= StereoBM::create(16, 9);
setParams();
punkt=Point2d(300, 300);
};

StereoCamera::~StereoCamera() {};

void StereoCamera::showMenu() {
	namedWindow("Camera and stereo settings");
	
	createTrackbar("Number of disparities", "Camera and stereo settings", &numdis, 16);
	createTrackbar("Window size", "Camera and stereo settings", &wsize, 40);
	createTrackbar("Uniqueness ratio", "Camera and stereo settings", &unique, 100);
	createTrackbar("Scale", "Camera and stereo settings", &ratio, 500);
	createTrackbar("Offset", "Camera and stereo settings", &offset, 400);
	createTrackbar("Exposure", "Camera and stereo settings", &exposure,1000);
	createTrackbar("Offset", "Camera and stereo settings", &offset, 400);
	createTrackbar("focal center", "Camera and stereo settings", &focalcenter, 100);
	createTrackbar("focal length", "Camera and stereo settings", &focallength, 100);
	createTrackbar("baseline", "Camera and stereo settings", &baseline, 100);
};

void StereoCamera::setParams() {
	numberOfDisparities = numdis * 16;
	SADWindowSize = wsize * 2 + 1;
	bm->setPreFilterCap(prefilter);
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(texturet);
	bm->setUniquenessRatio(unique);
	bm->setSpeckleWindowSize(speckleSize);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(12);
};

bool StereoCamera::setExtrinsics(String filename,float scale) {

		FileStorage fs1(filename, FileStorage::READ);
		if (!fs1.isOpened())
		{
			cout << "Failed to open " << endl;
			return 0;
		}
		fs1["R"] >> R;
		fs1["T"] >> T;
		fs1["E"] >> E;
		fs1["F"] >> F;
		fs1["Q"] >> Q;
		calculateQs(scale);
		return 1;
};

void StereoCamera::match(Mat frame1, Mat frame2) {
	calculateQs(0.2);
	bm->compute(frame1,frame2, disp);
};

void StereoCamera::calculateQs(float scale) {
	Qs = Q.clone();
	Qs.at<double>(0, 0) = 1.0;
	Qs.at<double>(0, 1) = 0.0;
	Qs.at<double>(0, 2) = 0.0;
	Qs.at<double>(0, 3) *= scale*(double)focalcenter/10; //cx
	Qs.at<double>(1, 0) = 0.0;
	Qs.at<double>(1, 1) = 1.0;
	Qs.at<double>(1, 2) = 0.0;
	Qs.at<double>(1, 3) *= scale*(double)focalcenter / 10;  //cy
	Qs.at<double>(2, 0) = 0.0;
	Qs.at<double>(2, 1) = 0.0;
	Qs.at<double>(2, 2) = 0.0;
	Qs.at<double>(2, 3) *= scale*(double)focallength / 10;  //Focal
	Qs.at<double>(3, 0) = 0.0;
	Qs.at<double>(3, 1) = 0.0;
	Qs.at<double>(3, 2) *= 1*(double)baseline / 10;    //1.0/BaseLine
	Qs.at<double>(3, 3) = 0.0;

	cout << "focalcenter" << scale*(double)focalcenter / 10 << endl;

	cout << "focallength" << scale*(double)focallength / 10 << endl;

	cout << "baseline" << (double)baseline / 10 << endl;
	//Qs.convertTo(Qs, CV_32F);
};

double StereoCamera::distCentralArea() {

	disp_size = disp.size();

	area_rect = Rect(disp_size.width / 2 - disp_size.width / 4, disp_size.height / 2 - disp_size.height / 4, disp_size.width / 2, disp_size.height / 2);

	area_h = Range(disp_size.height / 2 - disp_size.height / 4, disp_size.height / 2 + disp_size.height / 4);
	area_w = Range(disp_size.width / 2 - disp_size.width / 4, disp_size.width / 2 + disp_size.width / 4);
	disp.convertTo(disp, CV_32FC1);
	minMaxLoc(disp(area_h, area_w), &farthest_dist, &nearest_dist);
	//float d = disp.at<float>(punkt);
	centdistance = 1000* 0.2 * 0.001 / Q.at<double>(3, 2)*Q.at<double>(2, 3) / nearest_dist*16.f;
	nearest_dist = centdistance;
	ss << nearest_dist;
	text = ss.str();

	return centdistance;

}

double StereoCamera::avoidDirection() {
	disp_size = disp.size();
	border = 50;
	dir_area_l = Range(border, disp_size.width*0.5);
	dir_area_r = Range(disp_size.width*0.5, disp_size.width - border);
	dir_area_h = Range(disp_size.height*0.3, disp_size.height*0.9);
	sum_l_scalar = sum(disp(dir_area_h, dir_area_l));
	sum_l = sum_l_scalar[0] / countNonZero(disp(dir_area_h, dir_area_l));
	sum_r_scalar = sum(disp(dir_area_h, dir_area_r));
	sum_r = sum_r_scalar[0] / countNonZero(disp(dir_area_h, dir_area_r));
	turn = sum_l - sum_r;

	left_area = Rect(border, disp_size.height*0.3, disp_size.width*0.5 - border, disp_size.height*0.6);
	right_area = Rect(disp_size.width*0.5, disp_size.height*0.3, disp_size.width*0.5 - border, disp_size.height*0.6);
	return turn;
}

void StereoCamera::preparePreview() {
	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

	disp8.convertTo(preview, -1, double(ratio) / 50., offset - 200);

	drawDashboard();

	resize(disp8, disp8, Size(), 2, 2, INTER_LINEAR);
}

void StereoCamera::drawDashboard() {

	applyColorMap(preview, preview, COLORMAP_JET);
	rectangle(preview, area_rect, Scalar(255, 255, 200), 2, 8);
	rectangle(preview, left_area, Scalar(255, 50, 50), 2, 8);
	rectangle(preview, right_area, Scalar(0, 100, 255), 2, 8);
	putText(preview, text, Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 250, 255), 1, CV_AA, 0);
}
