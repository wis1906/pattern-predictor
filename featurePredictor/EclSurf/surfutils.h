#pragma once

#include "opencv2/core.hpp"
#include "ipoint.h"

#include <vector>

/*
// Convert image to single channel 32F
IplImage* getGray(const IplImage *img);
*/
//! Draw a single feature on the image
void drawIpoint(cv::Mat *img, Ipoint &ipt, int tailSize = 0);

//! Draw all the Ipoints in the provided vector
void drawIpoints(cv::Mat *img, std::vector<Ipoint> &ipts, int tailSize = 0);

//! Draw a Point at feature location
void drawPoint(cv::Mat *img, Ipoint &ipt);


//! Draw a Point at all features
void drawPoints(cv::Mat *img, std::vector<Ipoint> &ipts);


//! Round float to nearest integer
inline int fRound(float flt)
{
	return (int)floor(flt + 0.5f);
}