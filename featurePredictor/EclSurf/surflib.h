#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "integral.h"
#include "fasthessian.h"
#include "surf.h"
#include "ipoint.h"
#include "surfutils.h"

//! Library function builds vector of described interest points
inline void surfDetDes(
	cv::Mat *img,  /* image to find Ipoints in */
	std::vector<Ipoint> &ipts, /* reference to vector of Ipoints */
	bool upright = false, /* run in rotation invariant mode? */
	int octaves = OCTAVES, /* number of octaves to calculate */
	int intervals = INTERVALS, /* number of intervals per octave */
	int init_sample = INIT_SAMPLE, /* initial sampling step */
	float thres = THRES /* blob response threshold */)
{
	cv::Mat gray_img;
	cv::Mat int_img;
	
	if (img->channels() != 1)
	{
		cv::cvtColor(*img, gray_img, cv::COLOR_BGR2GRAY);
	}
	else
	{
		gray_img = *img;
	}
	
	cv::integral(gray_img, int_img, CV_32F);

	
	// Create Fast Hessian Object
	FastHessian fh(&int_img, ipts, octaves, intervals, init_sample, thres);


	// Extract interest points and store in vector ipts
	fh.getIpoints();


	// Create Surf Descriptor Object
	Surf des(&int_img, ipts);


	// Extract the descriptors for the ipts
	des.getDescriptors(upright);
}