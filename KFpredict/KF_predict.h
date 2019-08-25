#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


namespace Kalman_example
{
	class KalmanFilter
	{
	public:
		KalmanFilter(float x, float y);
		//构造函数
		Point2f	run(float x, float y);//求预测值函数

		void delay_msec(int ms);//延时函数

	private:
		Mat measurement_;
		cv::KalmanFilter KF_;//
		float x;
		float y;
		int ms;


	};

}




