#include "surfutils.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>
#include <time.h>

using namespace std;

//-------------------------------------------------------

static const int NCOLOURS = 8;
static const cv::Scalar COLOURS[] = { cv::Scalar(255,0,0), cv::Scalar(0,255,0),
									cv::Scalar(0,0,255), cv::Scalar(255,255,0),
									cv::Scalar(0,255,255), cv::Scalar(255,0,255),
									cv::Scalar(255,255,255), cv::Scalar(0,0,0) };

//-------------------------------------------------------
/*
// Convert image to single channel 32F
IplImage *getGray(const IplImage *img)
{
	// Check we have been supplied a non-null img pointer
	if (!img) error("Unable to create grayscale image.  No image supplied");

	IplImage* gray8, *gray32;

	gray32 = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);

	if (img->nChannels == 1)
		gray8 = (IplImage *)cvClone(img);
	else {
		gray8 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvCvtColor(img, gray8, CV_BGR2GRAY);
	}

	cvConvertScale(gray8, gray32, 1.0 / 255.0, 0);

	cvReleaseImage(&gray8);
	return gray32;
}
*/
//-------------------------------------------------------


//! Draw all the Ipoints in the provided vector
void drawIpoints(cv::Mat *img, vector<Ipoint> &ipts, int tailSize)
{
	Ipoint *ipt;
	float s, o;
	int r1, c1, r2, c2, lap;

	for (unsigned int i = 0; i < ipts.size(); i++)
	{
		ipt = &ipts.at(i);
		s = (2.5f * ipt->scale);
		o = ipt->orientation;
		lap = ipt->laplacian;
		r1 = fRound(ipt->y);
		c1 = fRound(ipt->x);
		c2 = fRound(s * cos(o)) + c1;
		r2 = fRound(s * sin(o)) + r1;

		if (o) // Green line indicates orientation
			cv::line(*img, cv::Point(c1, r1), cv::Point(c2, r2), cv::Scalar(0, 255, 0));
		else  // Green dot if using upright version
			cv::circle(*img, cv::Point(c1, r1), 1, cv::Scalar(0, 255, 0), -1);

		if (lap == 1)
		{ // Blue circles indicate dark blobs on light backgrounds
			cv::circle(*img, cv::Point(c1, r1), fRound(s), cv::Scalar(255, 0, 0), 1);
		}
		else if (lap == 0)
		{ // Red circles indicate light blobs on dark backgrounds
			cv::circle(*img, cv::Point(c1, r1), fRound(s), cv::Scalar(0, 0, 255), 1);
		}
		else if (lap == 9)
		{ // Red circles indicate light blobs on dark backgrounds
			cv::circle(*img, cv::Point(c1, r1), fRound(s), cv::Scalar(0, 255, 0), 1);
		}

		// Draw motion from ipoint dx and dy
		if (tailSize)
		{
			cv::line(*img, cv::Point(c1, r1),
				cv::Point(int(c1 + ipt->dx*tailSize), int(r1 + ipt->dy*tailSize)),
				cv::Scalar(255, 255, 255), 1);
		}
	}
}

//! Draw a single feature on the image
void drawIpoint(cv::Mat *img, Ipoint &ipt, int tailSize)
{
	float s, o;
	int r1, c1, r2, c2, lap;

	s = (2.5f * ipt.scale);
	o = ipt.orientation;
	lap = ipt.laplacian;
	r1 = fRound(ipt.y);
	c1 = fRound(ipt.x);

	// Green line indicates orientation
	if (o) // Green line indicates orientation
	{
		c2 = fRound(s * cos(o)) + c1;
		r2 = fRound(s * sin(o)) + r1;
		cv::line(*img, cv::Point(c1, r1), cv::Point(c2, r2), cv::Scalar(0, 255, 0));
	}
	else  // Green dot if using upright version
		cv::circle(*img, cv::Point(c1, r1), 1, cv::Scalar(0, 255, 0), -1);

	if (lap >= 0)
	{ // Blue circles indicate light blobs on dark backgrounds
		cv::circle(*img, cv::Point(c1, r1), fRound(s), cv::Scalar(255, 0, 0), 1);
	}
	else
	{ // Red circles indicate light blobs on dark backgrounds
		cv::circle(*img, cv::Point(c1, r1), fRound(s), cv::Scalar(0, 0, 255), 1);
	}

	// Draw motion from ipoint dx and dy
	if (tailSize)
	{
		cv::line(*img, cv::Point(c1, r1),
			cv::Point(int(c1 + ipt.dx*tailSize), int(r1 + ipt.dy*tailSize)),
			cv::Scalar(255, 255, 255), 1);
	}
}

//! Draw a single feature on the image
void drawPoint(cv::Mat *img, Ipoint &ipt)
{
	float s, o;
	int r1, c1;

	s = 3;
	o = ipt.orientation;
	r1 = fRound(ipt.y);
	c1 = fRound(ipt.x);

	cv::circle(*img, cv::Point(c1, r1), fRound(s), COLOURS[ipt.clusterIndex%NCOLOURS], -1);
	cv::circle(*img, cv::Point(c1, r1), fRound(s + 1), COLOURS[(ipt.clusterIndex + 1) % NCOLOURS], 2);

}

//! Draw a single feature on the image
void drawPoints(cv::Mat *img, vector<Ipoint> &ipts)
{
	float s, o;
	int r1, c1;

	for (unsigned int i = 0; i < ipts.size(); i++)
	{
		s = 3;
		o = ipts[i].orientation;
		r1 = fRound(ipts[i].y);
		c1 = fRound(ipts[i].x);

		cv::circle(*img, cv::Point(c1, r1), fRound(s), COLOURS[ipts[i].clusterIndex%NCOLOURS], -1);
		cv::circle(*img, cv::Point(c1, r1), fRound(s + 1), COLOURS[(ipts[i].clusterIndex + 1) % NCOLOURS], 2);
	}
}