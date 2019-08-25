#pragma once
#include <opencv2/opencv.hpp>

using namespace cv;


struct Pot
{



struct	Pot solve(int direct, double time, Point2d  circle, Point2d  current1, Point2d  current2, Point2d  current3, Point2d  current4);



	int dir;
	double t;
	Point2d  cir;
	Point2d  cur1;
	Point2d  cur2;
	Point2d  cur3;
	Point2d  cur4;
	
	Point2d  cnt1;
	Point2d  cnt2;
	Point2d  cnt3;
	Point2d  cnt4;
	

};

