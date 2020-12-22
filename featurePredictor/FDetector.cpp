// Fill out your copyright notice in the Description page of Project Settings.

#include "FDetector.h"

void FDetector::getObjFeaturePoint(Mat *img)
{
	surfDetDes(img, objIP, false, 4, 4, 2, 10.0f);
}
IpPairVec FDetector::matchImage(Mat *img)
{
	IpVec ipts;
	surfDetDes(img, ipts, false, 4, 4, 2, 10.0f);

	IpPairVec matches;
	matchWeight = getMatches(objIP, ipts, matches);


	return matches;
}

vector<cv::Point2f> FDetector::drawMatchedLine(Mat *img1, Mat *img2, IpPairVec matches)
{
	bool isDetected = true;
	rev.clear();

	vector<cv::Point2f> scene_corners(4);
	Mat H;
	scene_corners = translateCorners(img1, img2, matches, H);


	float cornerWidth1 =
		sqrt((scene_corners[0].x - scene_corners[1].x)*(scene_corners[0].x - scene_corners[1].x)
			+ (scene_corners[0].y - scene_corners[1].y)*(scene_corners[0].y - scene_corners[1].y));
	float cornerWidth2 =
		sqrt((scene_corners[2].x - scene_corners[3].x)*(scene_corners[2].x - scene_corners[3].x)
			+ (scene_corners[2].y - scene_corners[3].y)*(scene_corners[2].y - scene_corners[3].y));
	float cornerHeight1 =
		sqrt((scene_corners[1].x - scene_corners[2].x)*(scene_corners[1].x - scene_corners[2].x)
			+ (scene_corners[1].y - scene_corners[2].y)*(scene_corners[1].y - scene_corners[2].y));
	float cornerHeight2 =
		sqrt((scene_corners[3].x - scene_corners[0].x)*(scene_corners[3].x - scene_corners[0].x)
			+ (scene_corners[3].y - scene_corners[0].y)*(scene_corners[3].y - scene_corners[0].y));

	float cornerWidth = (cornerWidth1 + cornerWidth2) / 2;
	float cornerHeight = (cornerHeight1 + cornerHeight2) / 2;

	//UE_LOG(featurePredictor, Warning, TEXT("Corner : %f %f %f %f"), cornerWidth1, cornerWidth2, cornerHeight1, cornerHeight2);


	//너무 작으면 제외
	if (cornerWidth < img2->cols / 10)
		isDetected = false;
	else if (cornerHeight < img2->rows / 10)
		isDetected = false;
	//width와 height 비율차가 심하면 제외
	else if (cornerHeight / cornerWidth > 2)
		isDetected = false;
	else if (cornerWidth / cornerHeight > 2)
		isDetected = false;
	//width끼리, height끼리 길이 차가 심하면 제외
	else if (cornerWidth1 > cornerWidth2 * 3 || cornerWidth2 > cornerWidth1 * 3)
		isDetected = false;
	else if (cornerHeight1 > cornerHeight2 * 3 || cornerHeight2 > cornerHeight1 * 3)
		isDetected = false;


	//데이터가 들어오지 않은 것으로 보이면 제외
	if (H.rows == 0)
		isDetected = false;


	if (isDetected)
	{

		//UE_LOG(featurePredictor, Warning, TEXT("Detected Corner : %f %f %f %f"), cornerWidth1, cornerWidth2, cornerHeight1, cornerHeight2);
		//Get rotation of corners
		if (H.rows != 0)
		{
			//rev = decompHomography(H);
		}

		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		line(*img2, scene_corners[0], scene_corners[1], cv::Scalar(0, 0, 255), 7);
		line(*img2, scene_corners[1], scene_corners[2], cv::Scalar(0, 0, 255), 7);
		line(*img2, scene_corners[2], scene_corners[3], cv::Scalar(255, 0, 0), 7);
		line(*img2, scene_corners[3], scene_corners[0], cv::Scalar(0, 0, 255), 7);
	}
	else
	{
		scene_corners[0].x = -999;
		scene_corners[0].y = -999;
	}

	return scene_corners;
}