#include "ipoint.h"
#include <vector>
#include "opencv2/opencv.hpp"
#include <iostream>

//! Populate IpPairVec with matched ipts 
double getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches)
{
	double distSum = 0;
	int sumCnt = 0;
	float dist, d1, d2;
	Ipoint *match = NULL;

	matches.clear();

	for (unsigned int i = 0; i < ipts1.size(); i++)
	{
		d1 = d2 = FLT_MAX;

		for (unsigned int j = 0; j < ipts2.size(); j++)
		{
			dist = ipts1[i] - ipts2[j];

			if (dist < d1) // if this feature matches better than current best
			{
				d2 = d1;
				d1 = dist;
				match = &ipts2[j];
			}
			else if (dist < d2) // this feature matches better than second best
			{
				d2 = dist;
			}
		}

		// If match has a d1:d2 ratio < 0.65 ipoints are a match
		if (d1 / d2 < 0.65)
		{
			distSum += (double)1 - (double)d1 / (double)d2;
			sumCnt++;
			// Store the change in position
			ipts1[i].dx = match->x - ipts1[i].x;
			ipts1[i].dy = match->y - ipts1[i].y;
			matches.push_back(std::make_pair(ipts1[i], *match));
		}
	}

	return distSum;
}

std::vector<double> decompHomography(cv::Mat& H)
{
	float f = 100, w = 640, h = 480;

	cv::Mat1f K = (cv::Mat1f(3, 3) <<
		f, 0, w / 2,
		0, f, h / 2,
		0, 0, 1);

	std::vector<cv::Mat> Rs, Ts;

	customDecomposeHomographyMat(H, K, Rs, Ts);

	int size = Rs.size();

	std::vector<cv::Mat1d> rrvec(size);
	for (int i = 0; i < size; i++)
	{
		cv::Rodrigues(Rs[i], rrvec[i]);
		rrvec[i] = rrvec[i] * 180 / CV_PI;
	}


	std::vector<double> rev(size * 3);
	for (int i = 0; i < size * 3; i += 3)
	{
		rev[i] = rrvec[i / 3].at<double>(0);
		rev[i + 1] = rrvec[i / 3].at<double>(1);
		rev[i + 2] = rrvec[i / 3].at<double>(2);
	}

	return rev;
}

//! Find homography between matched points and translate src_corners to dst_corners
std::vector<cv::Point2f> translateCorners(cv::Mat *img_object, cv::Mat *img_matches, IpPairVec &matches, cv::Mat& H)
{
	double h[9];
	cv::Mat _h = cv::Mat(3, 3, CV_64F, h);
	std::vector<cv::Point2f> pt1, pt2;
	cv::Mat _pt1, _pt2;
	std::vector<cv::Point2f> obj_corners(4), scene_corners(4);

	int n = (int)matches.size();
	if (n < 4) return scene_corners;

	// Set vectors to correct size
	pt1.resize(n);
	pt2.resize(n);

	// Copy Ipoints from match vector into cvPoint vectors
	for (int i = 0; i < n; i++)
	{
		pt1[i] = cv::Point2f(matches[i].first.x, matches[i].first.y);
		pt2[i] = cv::Point2f(matches[i].second.x, matches[i].second.y);
	}

	H = cv::findHomography(pt1, pt2, cv::RANSAC);


	//UE_LOG(featurePredictor, Warning, TEXT("%d  %d"), H.rows, H.cols);
	if (H.rows == 0)
	{
		return scene_corners;
	}
	obj_corners[0] = cv::Point(0, 0); obj_corners[1] = cv::Point(img_object->cols, 0);
	obj_corners[2] = cv::Point(img_object->cols, img_object->rows); obj_corners[3] = cv::Point(0, img_object->rows);

	perspectiveTransform(obj_corners, scene_corners, H);

	return scene_corners;
}