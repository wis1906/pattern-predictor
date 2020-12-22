#include "decompHomography.h"

// function decomposes image-to-image homography to rotation and translation matrices
int customDecomposeHomographyMat(cv::InputArray _H,
	cv::InputArray _K,
	std::vector<cv::Mat>& _rotations,
	std::vector<cv::Mat>& _translations)
{
	using namespace std;

	cv::Mat H = _H.getMat().reshape(1, 3);
	CV_Assert(H.cols == 3 && H.rows == 3);

	cv::Mat K = _K.getMat().reshape(1, 3);
	CV_Assert(K.cols == 3 && K.rows == 3);

	cv::Ptr<HomographyDecomp> hdecomp(new HomographyDecomp);

	vector<CameraMotion> motions;
	hdecomp->customDecomposeHomography(H, K, motions);

	int nsols = static_cast<int>(motions.size());
	int depth = CV_64F; //double precision matrices used in CameraMotion struct

	_rotations = std::vector<cv::Mat>(nsols);
	for (int k = 0; k < nsols; ++k) {
		_rotations[k] = cv::Mat(motions[k].R);
	}

	_translations = std::vector<cv::Mat>(nsols);
	for (int k = 0; k < nsols; ++k) {
		_translations[k] = cv::Mat(motions[k].t);
	}


	return nsols;
}

//!main routine to decompose homography
void HomographyDecomp::customDecomposeHomography(const cv::Matx33d& H, const cv::Matx33d& K,
	std::vector<CameraMotion>& camMotions)
{
	//normalize homography matrix with intrinsic camera matrix
	_Hnorm = normalize(H, K);
	//remove scale of the normalized homography
	removeScale();
	//apply decomposition
	decompose(camMotions);
}

// normalizes homography with intrinsic camera parameters
cv::Matx33d HomographyDecomp::normalize(const cv::Matx33d& H, const cv::Matx33d& K)
{
	return K.inv() * H * K;
}

void HomographyDecomp::removeScale()
{
	cv::Mat W;
	cv::SVD::compute(_Hnorm, W);
	_Hnorm = _Hnorm * (1.0 / W.at<double>(1));
}

void HomographyDecomp::decompose(std::vector<CameraMotion>& camMotions)
{
	const double epsilon = 0.001;
	cv::Matx33d S;

	//S = H'H - I
	S = getHnorm().t() * getHnorm();
	S(0, 0) -= 1.0;
	S(1, 1) -= 1.0;
	S(2, 2) -= 1.0;

	//check if H is rotation matrix
	if (norm(S, cv::NORM_INF) < epsilon) {
		CameraMotion motion;
		motion.R = cv::Matx33d(getHnorm());
		motion.t = cv::Vec3d(0, 0, 0);
		motion.n = cv::Vec3d(0, 0, 0);
		camMotions.push_back(motion);
		return;
	}

	//! Compute nvectors
	cv::Vec3d npa, npb;

	double M00 = oppositeOfMinor(S, 0, 0);
	double M11 = oppositeOfMinor(S, 1, 1);
	double M22 = oppositeOfMinor(S, 2, 2);

	double rtM00 = sqrt(M00);
	double rtM11 = sqrt(M11);
	double rtM22 = sqrt(M22);

	double M01 = oppositeOfMinor(S, 0, 1);
	double M12 = oppositeOfMinor(S, 1, 2);
	double M02 = oppositeOfMinor(S, 0, 2);

	int e12 = signd(M12);
	int e02 = signd(M02);
	int e01 = signd(M01);

	double nS00 = abs(S(0, 0));
	double nS11 = abs(S(1, 1));
	double nS22 = abs(S(2, 2));

	//find max( |Sii| ), i=0, 1, 2
	int indx = 0;
	if (nS00 < nS11) {
		indx = 1;
		if (nS11 < nS22)
			indx = 2;
	}
	else {
		if (nS00 < nS22)
			indx = 2;
	}

	switch (indx) {
	case 0:
		npa[0] = S(0, 0), npb[0] = S(0, 0);
		npa[1] = S(0, 1) + rtM22, npb[1] = S(0, 1) - rtM22;
		npa[2] = S(0, 2) + e12 * rtM11, npb[2] = S(0, 2) - e12 * rtM11;
		break;
	case 1:
		npa[0] = S(0, 1) + rtM22, npb[0] = S(0, 1) - rtM22;
		npa[1] = S(1, 1), npb[1] = S(1, 1);
		npa[2] = S(1, 2) - e02 * rtM00, npb[2] = S(1, 2) + e02 * rtM00;
		break;
	case 2:
		npa[0] = S(0, 2) + e01 * rtM11, npb[0] = S(0, 2) - e01 * rtM11;
		npa[1] = S(1, 2) + rtM00, npb[1] = S(1, 2) - rtM00;
		npa[2] = S(2, 2), npb[2] = S(2, 2);
		break;
	default:
		break;
	}

	double traceS = S(0, 0) + S(1, 1) + S(2, 2);
	double v = 2.0 * sqrt(1 + traceS - M00 - M11 - M22);

	double ESii = signd(S(indx, indx));
	double r_2 = 2 + traceS + v;
	double nt_2 = 2 + traceS - v;

	double r = sqrt(r_2);
	double n_t = sqrt(nt_2);

	cv::Vec3d na = npa / norm(npa);
	cv::Vec3d nb = npb / norm(npb);

	double half_nt = 0.5 * n_t;
	double esii_t_r = ESii * r;

	cv::Vec3d ta_star = half_nt * (esii_t_r * nb - n_t * na);
	cv::Vec3d tb_star = half_nt * (esii_t_r * na - n_t * nb);

	camMotions.resize(4);

	cv::Matx33d Ra, Rb;
	cv::Vec3d ta, tb;

	//Ra, ta, na
	findRmatFrom_tstar_n(ta_star, na, v, Ra);
	ta = Ra * ta_star;

	camMotions[0].R = Ra;
	camMotions[0].t = ta;
	camMotions[0].n = na;

	//Ra, -ta, -na
	camMotions[1].R = Ra;
	camMotions[1].t = -ta;
	camMotions[1].n = -na;

	//Rb, tb, nb
	findRmatFrom_tstar_n(tb_star, nb, v, Rb);
	tb = Rb * tb_star;

	camMotions[2].R = Rb;
	camMotions[2].t = tb;
	camMotions[2].n = nb;

	//Rb, -tb, -nb
	camMotions[3].R = Rb;
	camMotions[3].t = -tb;
	camMotions[3].n = -nb;
}

double HomographyDecomp::oppositeOfMinor(const cv::Matx33d& M, const int row, const int col)
{
	int x1 = col == 0 ? 1 : 0;
	int x2 = col == 2 ? 1 : 2;
	int y1 = row == 0 ? 1 : 0;
	int y2 = row == 2 ? 1 : 2;

	return (M(y1, x2) * M(y2, x1) - M(y1, x1) * M(y2, x2));
}

//computes R = H( I - (2/v)*te_star*ne_t )
void HomographyDecomp::findRmatFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, const double v, cv::Matx33d& R)
{
	cv::Matx31d tstar_m = cv::Matx31d(tstar);
	cv::Matx31d n_m = cv::Matx31d(n);
	cv::Matx33d I(1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0);

	R = getHnorm() * (I - (2 / v) * tstar_m * n_m.t());
	if (cv::determinant(R) < 0)
	{
		R *= -1;
	}
}