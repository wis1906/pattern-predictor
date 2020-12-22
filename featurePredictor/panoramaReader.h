#pragma once
#include "opencv2/opencv.hpp"
#include "Kalman.h"

#include <iostream>
#include <fstream>
#include <string>
#include <io.h>

using namespace cv;
using namespace std;


class panoramaReader
{

public:
	panoramaReader();
	void BeginPlay();
	void Tick();

	//Editor Adjustable Field
public:
	int panoramaWidth = 2000;
	int panoramaHeight = 1000;

	string filePath = "../imgs/";
	string savePath = "../imgs/save/";
	string imagePath_Obj = "Pattern4_Low.png";
	string imagePath_Scene = "PatternVideo8.mp4";
	string filePath_TimeData = "TimeData.txt";
	string filePath_AccuracyData = "AccuracyData.txt";

	string detectionMode = "FeatureWithKalman";  //List : "None", "FeatureDetection", "FeatureWithKalman"
	string playMode = "Video";  //List : "Image", "Video"

	float kalman_CenterQ = 0.01f;
	float kalman_CenterR = 0.01f;
	float kalman_CenterVelocityAlpha = 0.4f;
	float kalman_CornerQ = 0.01f;
	float kalman_CornerR = 0.1f;
	float kalman_CornerVelocityAlpha = 0.4f;
	int newDetectCntNum = 3;
	int predictionScale = 3;

public:
	//flag
	bool isOpened = false;
	bool isFirstDetect = false;

	//Image Database
	class imageDatabase* imageData;

	//Feature detector
	class FDetector* detector;

	//Kalman Fillter
	pair<class Kalman*, class Kalman*> kalmanCenter;
	pair<class Kalman*, class Kalman*> kalmanCorner[4];
	bool kalmanInit = false;

	//Check Field for New Detection
	int notDetectedCnt = 0;
	char newDetectType[2] = { 0 , 0 };
	int newDetectCnt = 0;
	int newDetectWeight = 0;

	//File IO
	fstream TimedataFile;
	fstream AccuracydataFile;

	//For test value
	int detectSuccessCnt = 0;
	int detectFailedCnt = 0;

public:
	//For Init
	void initImageDatabase(imageDatabase* imageData, int width, int height);

	//For Processing
	void doProcessing(imageDatabase* imgData);

	//For feature detection
	void findMostMatchedType(imageDatabase* imgData);
	double findMatchedType(char type, imageDatabase* imgData);
	void detectFeatureByMatchedType(char c, imageDatabase* imgData);
	void detectFeature(char type, Mat *Img_Obj, Mat *Img_Scene);
	void drawLineDetectedImg(Mat *Img_Scene);

	//For Kalman Filter
	void updateKalman(vector<cv::Point2f> scene_corners);
	void addExpectedDataToKalman();
	void drawKalman(Mat* img);

	//Check Field for New Detection
	void checkWeightTwoField();
	void checkField(char type);
	void addNewDetectFieldType(char type, int first, int second);

	//For panorama to cubemap
	void createCubemap(imageDatabase* imgData);
	void cubeToImg(imageDatabase* imgData, Mat* Img, char Type);
	Point3d getPanoramaAxis(int x, int y, char Type, int CubeWidth, int Width, int Height);
	Point3d getThetaPhi(double x, double y, double z);

	//For File IO
	bool loadFile(imageDatabase* imgData);
	void showImages();
};

class imageDatabase
{

public:
	VideoCapture VCap;

	Size size;

	Mat objImg;
	Mat sourceImg;
	Mat panoramaImg;

	Mat TopImg;
	Mat BottomImg;
	Mat RearImg;
	Mat LeftImg;
	Mat FrontImg;
	Mat RightImg;
};

