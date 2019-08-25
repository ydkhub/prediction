#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include<math.h>
#include "Norgan.h"  
//#define  PI=3.1415926;

using namespace std;
using namespace cv;


Pot  Pot::solve(int direct, double time, Point2d  circle, Point2d  current1, Point2d  current2, Point2d  current3, Point2d  current4)
{
	dir = direct;
	t = time;
	cir = circle;
	cur1 = current1;
	cur2 = current2;
	cur3 = current3;
	cur4 = current4;
	//double system_t = 0.35;//系统延迟
	//double bullet_t;//子弹延迟
	double  distance_d = 8.000;
	double Rotate_angle;//旋转角度

	double cur1_xlength;
	double cur2_xlength;
	double cur3_xlength;
	double cur4_xlength;

	double cur1_ylength;
	double cur2_ylength;
	double cur3_ylength;
	double cur4_ylength;

	double coe1 = 3.07692308;
	double coe2 = 2.46153846;
	Point2d  pred1;//预测后点
	Point2d  pred2;
	Point2d  pred3;
	Point2d  pred4;

	Point2d  tran1;//过渡
	Point2d  tran2;
	Point2d  tran3;
	Point2d  tran4;

	Point2d  predf1;
	Point2d  predf2;
	Point2d  predf3;
	Point2d  predf4;
	//bullet_t = distance_d / s;
	//Rotate_angle = 60 * (system_t + bullet_t);
	Rotate_angle = 60 * t;
	
	double ro = (Rotate_angle*3.1415926) / 180;

	double  Initial_atanangle1 = atan2((cir.y*coe2 - cur1.y*coe2), (cur1.x*coe1 - cir.x*coe1));//转换为弧度顺时针
	double  Initial_atanangle2 = atan2((cir.y*coe2 - cur2.y*coe2), (cur2.x*coe1 - cir.x*coe1));//转换为弧度顺时针
	double  Initial_atanangle3 = atan2((cir.y*coe2 - cur3.y*coe2), (cur3.x*coe1 - cir.x*coe1));//转换为弧度顺时针
	double  Initial_atanangle4 = atan2((cir.y*coe2 - cur4.y*coe2), (cur4.x*coe1 - cir.x*coe1));//转换为弧度顺时针


	double  Initial_angle1 = atan2((cur1.x*coe1 - cir.x*coe1), (cur1.y*coe2 - cir.y*coe2));//转换为弧度逆时针
	double  Initial_angle2 = atan2((cur2.x*coe1 - cir.x*coe1), (cur2.y*coe2 - cir.y*coe2));//转换为弧度逆时针
	double  Initial_angle3 = atan2((cur3.x*coe1 - cir.x*coe1), (cur3.y*coe2 - cir.y*coe2));//转换为弧度逆时针
	double  Initial_angle4 = atan2((cur4.x*coe1 - cir.x*coe1), (cur4.y*coe2 - cir.y*coe2));//转换为弧度逆时针
	//double  Initial_angle =(Initial_atanangle * 180 )/ 3.1415926;//转换为旋转角度
	double  radius1 = sqrt(pow(cur1.y*coe2 - cir.y*coe2, 2) + pow(cur1.x*coe1 - cir.x*coe1, 2));
	double  radius2 = sqrt(pow(cur2.y*coe2 - cir.y*coe2, 2) + pow(cur2.x*coe1 - cir.x*coe1, 2));
	double  radius3 = sqrt(pow(cur3.y*coe2 - cir.y*coe2, 2) + pow(cur3.x*coe1 - cir.x*coe1, 2));
	double  radius4 = sqrt(pow(cur4.y*coe2 - cir.y*coe2, 2) + pow(cur4.x*coe1 - cir.x*coe1, 2));
	
   
	
	if (dir == 1)
	{
		cur1_xlength = cos(ro)*radius1;
		cur1_ylength = sin(ro)*radius1;
		tran1.x = cur1_xlength;
		tran1.y = cur1_ylength;
		pred1.x = tran1.x*cos(Initial_atanangle1) + tran1.y*sin(Initial_atanangle1) + cir.x*coe1;
		pred1.y = tran1.y*cos(Initial_atanangle1) - tran1.x*sin(Initial_atanangle1) + cir.y*coe2;
		predf1.x = pred1.x/coe1;
		predf1.y = pred1.y/coe2;

		cur2_xlength = cos(ro)*radius2;
		cur2_ylength = sin(ro)*radius2;
		tran2.x = cur2_xlength;
		tran2.y = cur2_ylength;
		pred2.x = tran2.x*cos(Initial_atanangle2) + tran2.y*sin(Initial_atanangle2) + cir.x*coe1;
		pred2.y = tran2.y*cos(Initial_atanangle2) - tran2.x*sin(Initial_atanangle2) + cir.y*coe2;
		predf2.x = pred2.x/coe1 ;
		predf2.y = pred2.y /coe2;

		cur3_xlength = cos(ro)*radius3;
		cur3_ylength = sin(ro)*radius3;
		tran3.x = cur3_xlength;
		tran3.y = cur3_ylength;
		pred3.x = tran3.x*cos(Initial_atanangle3) + tran3.y*sin(Initial_atanangle3) + cir.x*coe1;
		pred3.y = tran3.y*cos(Initial_atanangle3) - tran3.x*sin(Initial_atanangle3) + cir.y*coe2;
		predf3.x = pred3.x /coe1;
		predf3.y = pred3.y/coe2 ;

		cur4_xlength = cos(ro)*radius4;
		cur4_ylength = sin(ro)*radius4;
		tran4.x = cur4_xlength;
		tran4.y = cur4_ylength;
		pred4.x = tran4.x*cos(Initial_atanangle4) + tran4.y*sin(Initial_atanangle4) + cir.x*coe1;
		pred4.y = tran4.y*cos(Initial_atanangle4) - tran4.x*sin(Initial_atanangle4) + cir.y*coe2;
		predf4.x = pred4.x/coe1;
		predf4.y = pred4.y/coe2 ;


		
	}
	else if (dir == 2)
	{
		cur1_xlength = sin(ro)*radius1;
		cur1_ylength = cos(ro)*radius1;
		tran1.x = cur1_xlength;
		tran1.y = cur1_ylength;
		pred1.x = tran1.x*cos(Initial_angle1) + tran1.y*sin(Initial_angle1) + cir.x*coe1;
		pred1.y = tran1.y*cos(Initial_angle1) - tran1.x*sin(Initial_angle1) + cir.y*coe2;
		predf1.x = pred1.x/coe1;
		predf1.y = pred1.y/coe2;

		cur2_xlength = sin(ro)*radius2;
		cur2_ylength = cos(ro)*radius2;
		tran2.x = cur2_xlength;
		tran2.y = cur2_ylength;
		pred2.x = tran2.x*cos(Initial_angle2) + tran2.y*sin(Initial_angle2) + cir.x*coe1;
		pred2.y = tran2.y*cos(Initial_angle2) - tran2.x*sin(Initial_angle2) + cir.y*coe2;
		predf2.x = pred2.x/coe1;
		predf2.y = pred2.y/coe2;

		cur3_xlength = sin(ro)*radius3;
		cur3_ylength = cos(ro)*radius3;
		tran3.x = cur3_xlength;
		tran3.y = cur3_ylength;
		pred3.x = tran3.x*cos(Initial_angle3) + tran3.y*sin(Initial_angle3) + cir.x*coe1;
		pred3.y = tran3.y*cos(Initial_angle3) - tran3.x*sin(Initial_angle3) + cir.y*coe2;
		predf3.x = pred3.x/coe1;
		predf3.y = pred3.y /coe2;

		cur4_xlength = sin(ro)*radius4;
		cur4_ylength = cos(ro)*radius4;
		tran4.x = cur4_xlength;
		tran4.y = cur4_ylength;
		pred4.x = tran4.x*cos(Initial_angle4) + tran4.y*sin(Initial_angle4) + cir.x*coe1;
		pred4.y = tran4.y*cos(Initial_angle4) - tran4.x*sin(Initial_angle4) + cir.y*coe2;
		predf4.x = pred4.x/coe1;
		predf4.y = pred4.y/coe2;


	}

	struct Pot ret;

	ret.cnt1 = predf1;
	ret.cnt2 = predf2;
	ret.cnt3 = predf3;
	ret.cnt4 = predf4;

	
	cout << ret.cnt1 << endl;
	cout << ret.cnt2 << endl;
	cout << ret.cnt3 << endl;
	cout << ret.cnt4 << endl;

	return ret;
}






int main()
{
	
	Mat img = imread("E:/ex_picture/sample/cur/buff_1024/3_1.jpg");
	//Pot pt;
	//Pot r;
	struct  Pot ret;
	 ret.solve(2, 1.2, Point2d(187,168), Point2d(280,250), Point2d(259,279), Point2d(266,234), Point2d(244,264));
	//pt.solve(1, 1.5, Point2d(172, 163), Point2d(61.207), Point2d(56, 165), Point2d(84, 206), Point2d(79, 158)) ;
	circle(img, Point2d(187, 168), 104, Scalar(0, 0, 255));
	//circle(img, Point2d(806, 632), 3, Scalar(0, 255, 0), 2);    


	circle(img, Point2d(278.128, 82.7791), 3, Scalar(0, 255, 0), 2);
	circle(img, Point2d(293.703, 116.706), 3, Scalar(0, 255, 0), 2);
	circle(img, Point2d(261.628, 94.4783), 3, Scalar(0, 255, 0), 2);
	circle(img, Point2d(277.655, 129.903), 3, Scalar(0, 255, 0), 2);
	namedWindow("img");   
	imshow("img", img);
	waitKey(0);
	// cout << pt.sovle.ret << endl;
	//system("pause");


	return 0;

}

/*

Point2d  Rgan::solve()
{
	//double system_t = 0.35;//系统延迟
	//double bullet_t;//子弹延迟
	double  distance_d = 8.000;
	double Rotate_angle;//旋转角度
	double cur_xlength;
	double cur_ylength;
	
	Point2d  pred;//预测后点
	Point2d  tran;//过渡
	//bullet_t = distance_d / s;
	//Rotate_angle = 60 * (system_t + bullet_t);
	Rotate_angle = 60 * t;


	double  Initial_atanangle = atan2((cir.y - cur.y), (cur.x - cir.x));//转换为弧度
	double  Initial_angle = atan2((cur.x - cir.x), (cur.y - cir.y));//转换为弧度
																						//double  Initial_angle =(Initial_atanangle * 180 )/ 3.1415926;//转换为旋转角度
	double  radius = sqrt(pow(cur.y - cir.y, 2) + pow(cur.x - cir.x, 2));
	double ro = (Rotate_angle*3.1415926) / 180;


	if (dir == true)
	{
		cur_xlength = cos(ro)*radius;
		cur_ylength = sin(ro)*radius;
		tran.x = cur_xlength;
		tran.y = cur_ylength;
		pred.x = tran.x*cos(Initial_atanangle) + tran.y*sin(Initial_atanangle) + cir.x;
		pred.y = tran.y*cos(Initial_atanangle) - tran.x*sin(Initial_atanangle) + cir.y;
	

	}
	else if (dir == false)
	{
		cur_xlength = sin(ro)*radius;
		cur_ylength = cos(ro)*radius;
		tran.x = cur_xlength;
		tran.y = cur_ylength;
		pred.x = tran.x*cos(Initial_angle) + tran.y*sin(Initial_angle) + cir.x;
		pred.y = tran.y*cos(Initial_angle) - tran.x*sin(Initial_angle) + cir.y;
		
	}

	return pred;

}




int main()
{

	Mat img = imread("E:/ex_picture/sample/cur/buff_1024/2_1.jpg");
	Rgan pt;

	pt.predict(true, 1.5, Point2d(172,163), Point2d(70,184));
	circle(img, Point2d(172,163), 104, Scalar(0, 0, 255));
	//circle(img, Point2d(806, 632), 3, Scalar(0, 255, 0), 2);
	circle(img, pt.solve(), 3, Scalar(0, 255, 0), 2);
	cout << pt.solve() << endl;
	namedWindow("img");
	imshow("img", img);
	waitKey(0);


	return 0;

}
*/