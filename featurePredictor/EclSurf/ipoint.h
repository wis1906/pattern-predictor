#pragma once

#include <vector>
#include <math.h>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "decompHomography.h"

//-------------------------------------------------------

class Ipoint; // Pre-declaration
typedef std::vector<Ipoint> IpVec;
typedef std::vector<std::pair<Ipoint, Ipoint> > IpPairVec;

//-------------------------------------------------------

//! Ipoint operations
double getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches);
std::vector<double> decompHomography(cv::Mat& H);
std::vector<cv::Point2f> translateCorners(cv::Mat *img_object, cv::Mat *img_matches, IpPairVec &matches, cv::Mat& H);
//-------------------------------------------------------

class Ipoint {

public:

	//! Destructor
	~Ipoint() {};

	//! Constructor
	Ipoint() : orientation(0) {};

	//! Gets the distance in descriptor space between Ipoints
	float operator-(const Ipoint &rhs)
	{
		float sum = 0.f;
		for (int i = 0; i < 64; ++i)
			sum += (this->descriptor[i] - rhs.descriptor[i])*(this->descriptor[i] - rhs.descriptor[i]);
		return sqrt(sum);
	};

	//! Coordinates of the detected interest point
	float x, y;

	//! Detected scale
	float scale;

	//! Orientation measured anti-clockwise from +ve x-axis
	float orientation;

	//! Sign of laplacian for fast matching purposes
	int laplacian;

	//! Vector of descriptor components
	float descriptor[64];

	//! Placeholds for point motion (can be used for frame to frame motion analysis)
	float dx, dy;

	//! Used to store cluster index
	int clusterIndex;
};
