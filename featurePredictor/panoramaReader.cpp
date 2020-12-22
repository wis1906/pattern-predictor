#include "panoramaReader.h"
#include "FDetector.h"
#include <chrono>

panoramaReader::panoramaReader()
{
	//Init classes
	imageData = new imageDatabase();
	detector = new FDetector();

	//Init Kalman class

	kalmanCenter.first = new Kalman();
	kalmanCenter.second = new Kalman();
	for (int i = 0; i < 4; i++)
	{
		kalmanCorner[i].first = new Kalman();
		kalmanCorner[i].second = new Kalman();
	}
}
// Called when the game starts or when spawned
void panoramaReader::BeginPlay()
{
	//Open Data File
	string pathString = filePath;
	pathString.append(filePath_TimeData);
	TimedataFile.open(pathString, ios::out);
	pathString = filePath;
	pathString.append(filePath_AccuracyData);
	AccuracydataFile.open(pathString, ios::out);


	//Initialize
	initImageDatabase(imageData, panoramaWidth, panoramaHeight);

	//Load File
	if (!loadFile(imageData))
	{
		cout << "Image not found." << endl;
	}

	if (detectionMode.compare("None") != 0)
	{
		detector->getObjFeaturePoint(&imageData->objImg);
	}
	//Processing for image
	if (playMode.compare("Image") == 0)
	{
		doProcessing(imageData);
	}

}
// Called every frame
void panoramaReader::Tick()
{
	while (true)
	{
		//Process video frame per tick
		if (isOpened && playMode.compare("Video") == 0)
		{
			if (!imageData->VCap.read(imageData->sourceImg))
			{
				//Restart Video
				AccuracydataFile << "detectSuccess : " << detectSuccessCnt << "\n";
				AccuracydataFile << "detectFailed : " << detectFailedCnt << "\n";
				return;

				string pathString = filePath;
				pathString.append(imagePath_Scene);
				imageData->VCap = VideoCapture(pathString);
				imageData->VCap.set(CAP_PROP_FRAME_WIDTH, panoramaWidth);
				imageData->VCap.set(CAP_PROP_FRAME_HEIGHT, panoramaHeight);
				imageData->VCap.read(imageData->sourceImg);
			}

			doProcessing(imageData);

		}
	}
}

//###############################
// Processing
//###############################
void panoramaReader::doProcessing(imageDatabase* imgData)
{
	//Get panorama image
	resize(imgData->sourceImg, imgData->panoramaImg, Size(panoramaWidth, panoramaHeight));

	//Create Cubemap
	createCubemap(imageData);

	auto start = chrono::system_clock::now();

	//Feature detection
	if (detectionMode.compare("None") != 0)
	{
		if (detectionMode.compare("FeatureWithKalman") == 0)
		{
			if (notDetectedCnt > 10)
			{
				notDetectedCnt = 0;
				isFirstDetect = false;
			}

			if (!isFirstDetect)
			{
				findMostMatchedType(imgData);
				cout << detector->matchType << " Field is most matched" << endl;
			}
			else
			{
				checkField(detector->matchType);
			}


			if (newDetectCnt != 0)
			{
				checkWeightTwoField();

				if (newDetectCnt == 0)
				{
					if (newDetectWeight > newDetectCntNum / 2)
					{
						detector->matchType = newDetectType[1];

						newDetectType[0] = 0;
						newDetectType[1] = 0;
						newDetectCnt = 0;
						newDetectWeight = 0;
						kalmanInit = true;


						cout << "Field change :" << detector->matchType << endl;
					}
					else if (newDetectWeight == newDetectCntNum / 2)
					{
						newDetectCnt++;
					}
					else
					{
						newDetectType[0] = 0;
						newDetectType[1] = 0;
						newDetectCnt = 0;
						newDetectWeight = 0;

						cout << "Field change : Not" << endl;
					}
				}


			}
		}
		else
		{
			findMostMatchedType(imgData);

			cout << detector->matchType << " Field is most matched" << endl;
		}

		detectFeatureByMatchedType(detector->matchType, imgData);
	}

	auto end = chrono::system_clock::now();
	chrono::milliseconds delta = chrono::duration_cast<chrono::milliseconds>(end - start);

	TimedataFile << delta.count() << "\n";

	showImages();
}

//###############################
// Feature Detection
//###############################
void panoramaReader::findMostMatchedType(imageDatabase* imgData)
{
	detector->matchType = 0;

	char type = 0;
	double max = 0;
	double curCnt = 0;


	curCnt = findMatchedType('T', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'T';
	}

	curCnt = findMatchedType('B', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'B';
	}

	curCnt = findMatchedType('F', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'F';
	}

	curCnt = findMatchedType('A', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'A';
	}

	curCnt = findMatchedType('L', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'L';
	}

	curCnt = findMatchedType('R', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'R';
	}

	detector->matchType = type;
}
double panoramaReader::findMatchedType(char type, imageDatabase* imgData)
{
	Mat *Img_Scene = 0;
	switch (type)
	{
	case 'T':
		Img_Scene = &imgData->TopImg;
		break;
	case 'B':
		Img_Scene = &imgData->BottomImg;
		break;
	case 'A':
		Img_Scene = &imgData->RearImg;
		break;
	case 'L':
		Img_Scene = &imgData->LeftImg;
		break;
	case 'F':
		Img_Scene = &imgData->FrontImg;
		break;
	case 'R':
		Img_Scene = &imgData->RightImg;
		break;
	}

	drawLineDetectedImg(Img_Scene);

	IpPairVec matches = detector->matchImage(Img_Scene);
	//UE_LOG(featurePredictor, Warning, TEXT("%c  %d\n"), type, matches.size());

	return detector->matchWeight;
}
void panoramaReader::detectFeatureByMatchedType(char c, imageDatabase* imgData)
{
	switch (c)
	{
	case 'T':
		detectFeature('T', &imgData->objImg, &imgData->TopImg);
		break;
	case 'B':
		detectFeature('B', &imgData->objImg, &imgData->BottomImg);
		break;
	case 'A':
		detectFeature('A', &imgData->objImg, &imgData->RearImg);
		break;
	case 'L':
		detectFeature('L', &imgData->objImg, &imgData->LeftImg);
		break;
	case 'F':
		detectFeature('F', &imgData->objImg, &imgData->FrontImg);
		break;
	case 'R':
		detectFeature('R', &imgData->objImg, &imgData->RightImg);
		break;
	}

	AccuracydataFile << detector->matchWeight << "\n";
}
void panoramaReader::detectFeature(char type, Mat *Img_Obj, Mat *Img_Scene)
{
	vector<cv::Point2f> scene_corners(4);

	drawLineDetectedImg(Img_Scene);

	//Detect Features of each image and Match images
	IpPairVec matches = detector->matchImage(Img_Scene);



	if (matches.size() == 0)
		return;

	//Get and draw corners of outline
	scene_corners = detector->drawMatchedLine(Img_Obj, Img_Scene, matches);



	//Some is detected
	if (scene_corners[0].x != -999 && scene_corners[0].y != -999)
	{
		detectSuccessCnt++;
		//Calculate Center Axis
		Point2f centerAxis;
		centerAxis.x = ((scene_corners[0].x + scene_corners[1].x + scene_corners[2].x + scene_corners[3].x) / 4);
		centerAxis.y = ((scene_corners[0].y + scene_corners[1].y + scene_corners[2].y + scene_corners[3].y) / 4);
		//Real Axis Center
		circle(*Img_Scene, Point((int)centerAxis.x, (int)centerAxis.y), 60, Scalar(0, 0, 255), 5);

		if (detectionMode.compare("FeatureWithKalman") == 0)
		{
			//Kalman
			updateKalman(scene_corners);
			drawKalman(Img_Scene);
		}

		notDetectedCnt = 0;
	}
	//Not detected
	else
	{
		detectFailedCnt++;
		if (detectionMode.compare("FeatureWithKalman") == 0)
		{
			if (isFirstDetect && !kalmanInit)
			{
				//Repeat Kalman data
				addExpectedDataToKalman();
				drawKalman(Img_Scene);
			}
		}
		notDetectedCnt++;
	}
}
void panoramaReader::drawLineDetectedImg(Mat *img)
{
	line(*img, Point(0, 0), Point(img->cols, 0), cv::Scalar(0, 255, 0), 15);
	line(*img, Point(img->cols, 0), Point(img->cols, img->rows), cv::Scalar(0, 255, 0), 15);
	line(*img, Point(img->cols, img->rows), Point(0, img->rows), cv::Scalar(0, 255, 0), 15);
	line(*img, Point(0, img->rows), Point(0, 0), cv::Scalar(0, 255, 0), 15);
}

//###############################
// Kalman Fillter
//###############################
void panoramaReader::updateKalman(vector<cv::Point2f> scene_corners)
{
	//Calculate Center Axis
	Point2f centerAxis;
	centerAxis.x = ((scene_corners[0].x + scene_corners[1].x + scene_corners[2].x + scene_corners[3].x) / 4);
	centerAxis.y = ((scene_corners[0].y + scene_corners[1].y + scene_corners[2].y + scene_corners[3].y) / 4);

	if (!isFirstDetect || kalmanInit)
	{
		cout << "Initialize Kalman Data" << endl;
		isFirstDetect = true;
		kalmanInit = false;

		//init center
		kalmanCenter.first->initKalman((double)centerAxis.x, kalman_CenterQ, 1, kalman_CenterR, kalman_CenterVelocityAlpha);
		kalmanCenter.second->initKalman((double)centerAxis.y, kalman_CenterQ, 1, kalman_CenterR, kalman_CenterVelocityAlpha);

		//init corners
		for (int i = 0; i < 4; i++)
		{
			kalmanCorner[i].first->initKalman((double)scene_corners[i].x - (double)centerAxis.x, kalman_CornerQ, 1, kalman_CornerR, kalman_CornerVelocityAlpha);
			kalmanCorner[i].second->initKalman((double)scene_corners[i].y - (double)centerAxis.y, kalman_CornerQ, 1, kalman_CornerR, kalman_CornerVelocityAlpha);
		}
	}
	else
	{
		//Update center
		kalmanCenter.first->KalmanPredictUpdate((double)centerAxis.x);
		kalmanCenter.first->setVelocity(kalmanCenter.first->X);
		kalmanCenter.second->KalmanPredictUpdate((double)centerAxis.y);
		kalmanCenter.second->setVelocity(kalmanCenter.second->X);

		//Update corners
		for (int i = 0; i < 4; i++)
		{
			kalmanCorner[i].first->KalmanPredictUpdate((double)scene_corners[i].x - (double)centerAxis.x);
			kalmanCorner[i].first->setVelocity(kalmanCorner[i].first->X);
			kalmanCorner[i].second->KalmanPredictUpdate((double)scene_corners[i].y - (double)centerAxis.y);
			kalmanCorner[i].second->setVelocity(kalmanCorner[i].second->X);
		}
	}
}
void panoramaReader::addExpectedDataToKalman()
{
	//Update center
	kalmanCenter.first->KalmanPredictUpdate(kalmanCenter.first->X + kalmanCenter.first->velocityX);
	kalmanCenter.second->KalmanPredictUpdate(kalmanCenter.second->X + kalmanCenter.second->velocityX);

	//Update corners
	for (int i = 0; i < 4; i++)
	{
		kalmanCorner[i].first->KalmanPredictUpdate(kalmanCorner[i].first->X + kalmanCenter.first->velocityX);
		kalmanCorner[i].second->KalmanPredictUpdate(kalmanCorner[i].second->X + kalmanCenter.second->velocityX);
	}
}
void panoramaReader::drawKalman(Mat* img)
{
	//Kalman Center Point
	circle(*img, Point(kalmanCenter.first->X, kalmanCenter.second->X), 40, Scalar(255, 255, 255), 5);

	//Expected Next Center Point
	circle(*img, Point(kalmanCenter.first->X + kalmanCenter.first->velocityX, kalmanCenter.second->X + kalmanCenter.second->velocityX), 20, Scalar(255, 0, 0), -1);

	//Draw Kalman Line
	for (int i = 0; i < 4; i++)
	{
		line(*img,
			Point(kalmanCorner[i % 4].first->X + kalmanCenter.first->X, kalmanCorner[i % 4].second->X + kalmanCenter.second->X),
			Point(kalmanCorner[(i + 1) % 4].first->X + kalmanCenter.first->X, kalmanCorner[(i + 1) % 4].second->X + kalmanCenter.second->X),
			cv::Scalar(255, 255, 255),
			7);
	}
}

//###############################
// Check Field for New Detection
//###############################
void panoramaReader::checkWeightTwoField()
{
	char type = 0;
	double max = 0;
	double curCnt = 0;

	curCnt = findMatchedType(newDetectType[0], imageData);
	if (max < curCnt)
	{
		max = curCnt;
		type = newDetectType[0];
	}

	curCnt = findMatchedType(newDetectType[1], imageData);
	if (max < curCnt)
	{
		max = curCnt;
		type = newDetectType[1];
	}

	if (type != 0 && type != newDetectType[0])
	{
		newDetectWeight++;
	}
	newDetectCnt--;

	//UE_LOG(featurePredictor, Warning, TEXT("Count : %d, Weight : %d"), newDetectCnt, newDetectWeight);
}
void panoramaReader::checkField(char type)
{

	if (kalmanCenter.first->velocityX < 0)
	{
		int expectedPos = kalmanCenter.first->X + kalmanCenter.first->velocityX * predictionScale;

		if (expectedPos < 0)
		{
			addNewDetectFieldType(type, 0, 0);
			return;
		}
	}
	else if (kalmanCenter.first->velocityX >= 0)
	{
		int expectedPos = kalmanCenter.first->X + kalmanCenter.first->velocityX * predictionScale;

		if (expectedPos > imageData->TopImg.cols)
		{
			addNewDetectFieldType(type, 0, 1);
			return;
		}
	}
	if (kalmanCenter.second->velocityX < 0)
	{
		int expectedPos = kalmanCenter.second->X + kalmanCenter.second->velocityX * predictionScale;

		if (expectedPos < 0)
		{
			addNewDetectFieldType(type, 1, 0);
			return;
		}
	}
	else if (kalmanCenter.second->velocityX >= 0)
	{
		int expectedPos = kalmanCenter.second->X + kalmanCenter.second->velocityX * predictionScale;

		if (expectedPos > imageData->TopImg.rows)
		{
			addNewDetectFieldType(type, 1, 1);
			return;
		}
	}
}
void panoramaReader::addNewDetectFieldType(char type, int first, int second)
{
	char checkType[2] = { type , 0 };

	char fieldT[2][2] = { {'L','R' },{'A', 'F'} };
	char fieldB[2][2] = { {'L','R' },{'F', 'A'} };
	char fieldF[2][2] = { {'L','R' },{'T', 'B'} };
	char fieldA[2][2] = { {'R','L' },{'T', 'B'} };
	char fieldL[2][2] = { {'A','F' },{'T', 'B'} };
	char fieldR[2][2] = { {'F','A' },{'T', 'B'} };

	switch (type)
	{
	case 'T':
		checkType[1] = fieldT[first][second];
		break;
	case 'B':
		checkType[1] = fieldB[first][second];
		break;
	case 'A':
		checkType[1] = fieldA[first][second];
		break;
	case 'L':
		checkType[1] = fieldL[first][second];
		break;
	case 'F':
		checkType[1] = fieldF[first][second];
		break;
	case 'R':
		checkType[1] = fieldR[first][second];
		break;
	}

	if (newDetectType[0] == checkType[0] && newDetectType[1] == checkType[1])
	{
		return;
	}


	cout << "Check : [ " << checkType[0] << ", " << checkType[1] << " ] Field" << endl;
	newDetectType[0] = checkType[0];
	newDetectType[1] = checkType[1];
	newDetectCnt = newDetectCntNum;
	newDetectWeight = 0;
}

//###############################
// Panorama to Cubemap
//###############################
void panoramaReader::createCubemap(imageDatabase* imgData)
{
	cubeToImg(imgData, &imgData->RearImg, 'A');
	cubeToImg(imgData, &imgData->LeftImg, 'L');
	cubeToImg(imgData, &imgData->FrontImg, 'F');
	cubeToImg(imgData, &imgData->RightImg, 'R');
	cubeToImg(imgData, &imgData->TopImg, 'T');
	cubeToImg(imgData, &imgData->BottomImg, 'B');
}
void panoramaReader::cubeToImg(imageDatabase* imgData, Mat* Img, char Type)
{
	int width = Img->cols;
	int height = Img->rows;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			Point3d xy = getPanoramaAxis(j, i, Type, width, imgData->panoramaImg.cols, imgData->panoramaImg.rows);


			int idx = j + (i * width);
			int xy_idx = (int)xy.x + ((int)xy.y * panoramaWidth);

			Img->data[idx * 3 + 0] = imgData->panoramaImg.data[xy_idx * 3 + 0];
			Img->data[idx * 3 + 1] = imgData->panoramaImg.data[xy_idx * 3 + 1];
			Img->data[idx * 3 + 2] = imgData->panoramaImg.data[xy_idx * 3 + 2];
		}
	}
}
Point3d panoramaReader::getPanoramaAxis(int x, int y, char Type, int CubeWidth, int Width, int Height)
{
	double u = 2 * (double(x) / CubeWidth - 0.5f);
	double v = 2 * (double(y) / CubeWidth - 0.5f);

	Point3d ThetaPhi;
	if (Type == 'F')
	{
		ThetaPhi = getThetaPhi(1, u, -v);
	}
	else if (Type == 'R')
	{
		ThetaPhi = getThetaPhi(-u, 1, -v);
	}
	else if (Type == 'L')
	{
		ThetaPhi = getThetaPhi(u, -1, -v);
	}
	else if (Type == 'A')
	{
		ThetaPhi = getThetaPhi(-1, -u, -v);
	}
	else if (Type == 'B')
	{
		ThetaPhi = getThetaPhi(-v, u, -1);
	}
	else if (Type == 'T')
	{
		ThetaPhi = getThetaPhi(v, u, 1);
	}

	double _x;
	double _y;
	_x = 0.5 + 0.5*(ThetaPhi.x / CV_PI);
	_y = (ThetaPhi.y / CV_PI);

	Point3d xy;
	xy.x = _x * Width;
	xy.y = _y * Height;
	if (xy.x >= Width)
	{
		xy.x = Width - 1;
	}
	if (xy.y >= Height)
	{
		xy.y = Height - 1;
	}
	return xy;
}
Point3d panoramaReader::getThetaPhi(double x, double y, double z)
{
	double dv = sqrt(x * x + y * y + z * z);
	x = x / dv;
	y = y / dv;
	z = z / dv;

	Point3d ThetaPhi;
	ThetaPhi.x = atan2(y, x);
	ThetaPhi.y = acos(z);
	return ThetaPhi;
}

//###############################
// File IO
//###############################
bool panoramaReader::loadFile(imageDatabase* imgData)
{
	string pathString;

	isOpened = true;

	//Load object image
	pathString = filePath;
	pathString.append(imagePath_Obj);
	imgData->objImg = imread(pathString);
	if (imgData->objImg.empty())
	{
		isOpened = false;
	}

	if (playMode.compare("Video") == 0)
	{

		pathString = filePath;
		pathString.append(imagePath_Scene);
		imgData->VCap = VideoCapture(pathString);
		if (!imgData->VCap.isOpened())
		{
			isOpened = false;
		}
		else
		{
			//Adjust Width and Height of Video
			imgData->VCap.set(CAP_PROP_FRAME_WIDTH, panoramaWidth);
			imgData->VCap.set(CAP_PROP_FRAME_HEIGHT, panoramaHeight);
		}
	}
	else
	{
		//Load scene image
		pathString = filePath;
		pathString.append(imagePath_Scene);
		imgData->sourceImg = imread(pathString);
		if (imgData->sourceImg.empty())
		{
			isOpened = false;
		}
	}

	return isOpened;
}
void panoramaReader::showImages()
{
	int sz = 300;

	Mat PanoramaImg;
	Mat RearImg;
	Mat LeftImg;
	Mat FrontImg;
	Mat RightImg;
	Mat TopImg;
	Mat BottomImg;

	resize(imageData->panoramaImg, PanoramaImg, Size(sz * 4, sz * 2));
	resize(imageData->RearImg, RearImg, Size(sz, sz));
	resize(imageData->LeftImg, LeftImg, Size(sz, sz));
	resize(imageData->FrontImg, FrontImg, Size(sz, sz));
	resize(imageData->RightImg, RightImg, Size(sz, sz));
	resize(imageData->TopImg, TopImg, Size(sz, sz));
	resize(imageData->BottomImg, BottomImg, Size(sz, sz));


	//imshow("Panorama", PanoramaImg);
	imshow("Bottom", BottomImg);
	moveWindow("Bottom", sz + 2 * sz, sz + sz);
	imshow("Rear", RearImg);
	moveWindow("Rear", sz, sz);
	imshow("Left", LeftImg);
	moveWindow("Left", sz + 1 * sz, sz);
	imshow("Front", FrontImg);
	moveWindow("Front", sz + 2 * sz, sz);
	imshow("Right", RightImg);
	moveWindow("Right", sz + 3 * sz, sz);
	imshow("Top", TopImg);
	moveWindow("Top", sz + 2 * sz, sz - sz);

	string pathString = savePath;
	pathString.append("panorama.jpg");
	imwrite(pathString, PanoramaImg);
	pathString = savePath;
	pathString.append("top.jpg");
	imwrite(pathString, TopImg);
	pathString = savePath;
	pathString.append("bottom.jpg");
	imwrite(pathString, BottomImg);
	pathString = savePath;
	pathString.append("left.jpg");
	imwrite(pathString, LeftImg);
	pathString = savePath;
	pathString.append("right.jpg");
	imwrite(pathString, RightImg);
	pathString = savePath;
	pathString.append("rear.jpg");
	imwrite(pathString, RearImg);
	pathString = savePath;
	pathString.append("front.jpg");
	imwrite(pathString, FrontImg);

	waitKey(1);
}
//###############################
// Initialize
//###############################
void panoramaReader::initImageDatabase(imageDatabase* imgData, int width, int height)
{
	Size size = Size((int)width / 4, (int)height / 2);
	imgData->size = size;

	//Init Image
	imgData->TopImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->BottomImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->RearImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->LeftImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->FrontImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->RightImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
}
