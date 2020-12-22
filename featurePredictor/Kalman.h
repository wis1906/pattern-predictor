#pragma once

class Kalman
{
public:
	//ID
	char type = 0;

	//Velocity
	double velocityAlpha = 0.4f;
	double prevX = 0;
	double prevVelocityX = 0;
	double velocityX = 0;

	//Kalman
	double	X = 0;  // 상태 행렬
	double	Q = 0.001f;  // 센서 노이즈 공분산 상수
	double	P = 1;  // 상태 공분산 행렬
	double	R = 0.1f;  // 측정 공분산 행렬

public:
	void initKalman(double _X, double _Q, double _P, double _R, double _velocityAlpha);
	double KalmanPredictUpdate(double NewData);
	void setVelocity(double NewData);
};
