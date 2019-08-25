#include<KF_predict.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

 Kalman_example::KalmanFilter::KalmanFilter(float x, float y):
	KF_(4, 2)

	/*
	KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F )
	"dynamParams = 4": 4*1 vector of state (x, y, delta x, delta y)
	"measureParams = 2": 2*1 vector of measurement (x, y)
	*/
{
	measurement_ = Mat::zeros(2, 1, CV_32F);// 定义测量值
											//以下为卡尔曼滤波矩阵的设置，值不同回归效果不同
	KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,//**后者更大更快的回归，转移矩阵
		0, 1, 0, 1,//**更大更快的回归
		0, 0, 1, 0,
		0, 0, 0, 1);
	setIdentity(KF_.measurementMatrix, Scalar::all(1));//测量矩阵
	setIdentity(KF_.processNoiseCov, Scalar::all(1e-6));//**10:回归越大，越慢，系统噪声方差矩阵Q//系统误差
	setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-2));//1: ，测量噪声方差矩阵R，测量误差
	setIdentity(KF_.errorCovPost, Scalar::all(1));   //后验错误估计协方差矩阵P  修正的最小均方误差


	KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);//确保刚开始是默认值，确定初始状态
}

Point2f Kalman_example::KalmanFilter::run(float x, float y)//卡尔曼预测点函数
{
	  
	  float size_x = 1280;//cols of side
	  float size_y = 480;//rows of side
	  float anti_range = 0.5;
	
	  Point2f currentPoint = Point2f(x, y);
	
		Mat prediction = KF_.predict();//计算预测值，返回x
		Point2f predict_pt = Point2f(prediction.at<float>(0), prediction.at<float>(1));

		measurement_.at<float>(0, 0) = x;//测量值
		measurement_.at<float>(1, 0) = y;

		KF_.correct(measurement_);

		Point2f kalmanPoint = predict_pt;
		Point2f anti_kalmanPoint(x, y);
		

		if ((currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) <= size_x
			|| (currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) >= 0)//Prevent Anti-kal out of Mat
		{
			if (abs(currentPoint.x - kalmanPoint.x) > 3)//当目标点逐渐停止运动时，减少卡尔曼滤波的震动
				anti_kalmanPoint.x = currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x);
			else
				anti_kalmanPoint.x = currentPoint.x;
		}
		else
		{
			anti_kalmanPoint.x = currentPoint.x;
		}


		if ((currentPoint.y + anti_range*(currentPoint.y - kalmanPoint.y)) <= size_y
			|| (currentPoint.y + anti_range*(currentPoint.y - kalmanPoint.y)) >= 0)//Prevent Anti-kal out of Mat
		{
			if (abs(currentPoint.y - kalmanPoint.y) > 3)//当目标点逐渐停止运动时，减少卡尔曼滤波的震动
				anti_kalmanPoint.y = currentPoint.y + anti_range*(currentPoint.y - kalmanPoint.y);
			else
				anti_kalmanPoint.y = currentPoint.y;
		}
		else
		{
			anti_kalmanPoint.y = currentPoint.y;
		}
		return anti_kalmanPoint;
	
}

		
	

	void Kalman_example::KalmanFilter::delay_msec(int msec)
	{
		clock_t now = clock();
		while (clock() - now < msec);
	}




   

	int main()
	{
		float x = 0;//CV_32F: float
		float y = 0;//CV_32F: float
		Kalman_example::KalmanFilter kf(x, y);

		VideoCapture capture(0);
		while (1)

		{
			char c = 0;
			Mat frame;
			capture >> frame;
			Mat image(1280, 480, CV_8UC3);
			double TargetLost_times = 0;
			//Mat变量定义   
			Mat midImage;//目标图的定义
						 //转为灰度图并进行图像平滑
			cvtColor(frame, midImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图
			GaussianBlur(midImage, midImage, Size(9, 9), 2, 2);

			//【进行霍夫圆变换
			vector<Vec3f> circles;
			HoughCircles(midImage, circles, CV_HOUGH_GRADIENT, 1, 10, 200, 75, 0, 0);//改动第七个参数，改变圆心累加器的阈值，用来调整检测效果

			for (size_t i = 0; i < circles.size(); i++)
			{

				Point2f center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				float radius = cvRound(circles[i][2]);
				Point2f point;
				point.x = cvRound(circles[i][0]);
				point.y = cvRound(circles[i][1]);
				if (abs(radius) >= 1e-6 || abs(point.x) >= 1e-6 || abs(point.y) >= 1e-6)//float遵循R32-24所以float的精度误差在1e - 6；
																						//这里来判断当检测的目标点值小于1e - 6，即为0时判断目标丢失

				{
					circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);

					circle(frame, center, radius, Scalar(155, 50, 255), 3, 8, 0);

					kf.delay_msec(100);

					kf.run(point.x, point.y);

					circle(image, kf.run(point.x, point.y), 3, Scalar(0, 255, 0), 2);//绿色为预测点
					circle(image, point, 3, Scalar(255, 0, 0), 2);//红色为实际点
					imshow("Anti-KalmanPoint", image);
					cout << "Current: " << point << " Anti-Kalman: " << kf.run(point.x, point.y) << endl;
				}
				else
				{
					TargetLost_times++;

					if (TargetLost_times > 60)//长时间丢失目标下初始化滤波器
					{
						cv::KalmanFilter();
					}
				}

			}

			namedWindow("效果");
			imshow("效果", frame);
			c = cvWaitKey(10);

		}
		return 0;
	}