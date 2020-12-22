#pragma once

#include "EclSurf/surflib.h"

using namespace cv;
using namespace std;

class FDetector
{

public:
	IpVec objIP;

	char matchType = 0;
	std::vector<double> rev;
	double matchWeight = 0;

public:
	void getObjFeaturePoint(Mat *img);
	IpPairVec matchImage(Mat *img);
	vector<cv::Point2f> drawMatchedLine(Mat *img1, Mat *img2, IpPairVec matches);
};
