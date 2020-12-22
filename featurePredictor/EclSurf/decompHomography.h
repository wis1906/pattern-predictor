#pragma once

#include "opencv2/core/utility.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/ocl.hpp"
#include <iostream>

#define GET_OPTIMIZED(func) (func)

typedef struct _CameraMotion {
	cv::Matx33d R; //!< rotation matrix
	cv::Vec3d n; //!< normal of the plane the camera is looking at
	cv::Vec3d t; //!< translation vector
} CameraMotion;


class HomographyDecomp {

public:
	virtual void customDecomposeHomography(const cv::Matx33d& H, const cv::Matx33d& K,
		std::vector<CameraMotion>& camMotions);
	bool isRotationValid(const cv::Matx33d& R, const double epsilon = 0.01);

	bool passesSameSideOfPlaneConstraint(CameraMotion& motion);
	virtual void decompose(std::vector<CameraMotion>& camMotions);

	const cv::Matx33d& getHnorm() const {
		return _Hnorm;
	}

	cv::Matx33d normalize(const cv::Matx33d& H, const cv::Matx33d& K);
	void removeScale();
	cv::Matx33d _Hnorm;

	double oppositeOfMinor(const cv::Matx33d& M, const int row, const int col);
	void findRmatFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, const double v, cv::Matx33d& R);
};

inline int signd(const double x)
{
	return (x >= 0 ? 1 : -1);
}



template<typename T> inline int compressElems(T* ptr, const uchar* mask, int mstep, int count)
{
	int i, j;
	for (i = j = 0; i < count; i++)
		if (mask[i*mstep])
		{
			if (i > j)
				ptr[j] = ptr[i];
			j++;
		}
	return j;
}

static inline bool haveCollinearPoints(const cv::Mat& m, int count)
{
	int j, k, i = count - 1;
	const cv::Point2f* ptr = m.ptr<cv::Point2f>();

	// check that the i-th selected point does not belong
	// to a line connecting some previously selected points
	// also checks that points are not too close to each other
	for (j = 0; j < i; j++)
	{
		double dx1 = ptr[j].x - ptr[i].x;
		double dy1 = ptr[j].y - ptr[i].y;
		for (k = 0; k < j; k++)
		{
			double dx2 = ptr[k].x - ptr[i].x;
			double dy2 = ptr[k].y - ptr[i].y;
			if (fabs(dx2*dy1 - dy2 * dx1) <= FLT_EPSILON * (fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
				return true;
		}
	}
	return false;
}

int customDecomposeHomographyMat(cv::InputArray _H,
	cv::InputArray _K,
	std::vector<cv::Mat>& _rotations,
	std::vector<cv::Mat>& _translations);