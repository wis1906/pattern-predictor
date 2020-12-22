#pragma once

#include <algorithm>  // req'd for std::min/max

// undefine VS macros
#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#include "opencv2/core.hpp"
//! Computes the sum of pixels within the rectangle specified by the top-left start
//! co-ordinate and size
inline float BoxIntegral(cv::Mat *img, int row, int col, int rows, int cols)
{
	float *data = (float *)img->data;
	int step = (img->step) / sizeof(float);

	// The subtraction by one for row/col is because row/col is inclusive.
	int r1 = std::min(row, img->rows) - 1;
	int c1 = std::min(col, img->cols) - 1;
	int r2 = std::min(row + rows, img->rows) - 1;
	int c2 = std::min(col + cols, img->cols) - 1;

	float A(0.0f), B(0.0f), C(0.0f), D(0.0f);
	if (r1 >= 0 && c1 >= 0) A = data[r1 * step + c1];
	if (r1 >= 0 && c2 >= 0) B = data[r1 * step + c2];
	if (r2 >= 0 && c1 >= 0) C = data[r2 * step + c1];
	if (r2 >= 0 && c2 >= 0) D = data[r2 * step + c2];

	return std::max(0.f, A - B - C + D);
}