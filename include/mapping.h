#pragma once
#include <iostream>
#include <stdio.h>
#include <ctime>
#include "common.h"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "tracking.h"


#define MY_DISTANCE 300.0
#define LEFT 1
#define RIGHT 0

using namespace cv;
using namespace std;


class PerspectiveMapping
{
public:

    struct JiaoDian {

        Point jiaodian;

        double angle;

    };
      

        
      vector<Point> PointsEdgeLeft;     // 赛道左边缘点集
      vector<Point> PointsEdgeRight;    // 赛道右边缘点集
      vector<Point> testpoints;     // 赛道左边缘点集
      JiaoDian Left_jiaodian;
      JiaoDian Right_jiaodian;

      //0为右边，默认为右边
      bool direction = RIGHT;

      PerspectiveMapping();


      double PixelPitch();

      Point Cornerdetection(const Point point);
       /**
    * @brief  将vector透视变换后返回
    *
    * @param vector
    *
    * @return 
    */
      vector<Point> Cornerdetection(const vector<Point> Points);

      Point InverseCornerDetection(const cv::Point& transformedPoint);


        /**
    * @brief  找角点
    *
    * @param vector
    *
    * @return 角点返回
    */
      void findPointsjiaodian();

      void drawImage(Mat& trackImage);

      void Edgehandling(Tracking& tracking);

      void findSingleSideJiaodian(const vector<Point>& edgePoints,JiaoDian& output,bool isLeft,int windowSize,double minAngleDeg);

      Mat get_m_H();

      Mat m_H_inv;

private:

    //透视变换矩阵
    Mat m_H = (cv::Mat_<double>(3, 3) <<
        2.812499999999994, 12.53906249999997, 83.75000000000071,
    -1.740597407439278e-15, 19.68749999999996, 308.7500000000004,
    -3.141197189824225e-18, 0.02343749999999995, 1);
    

    //图片像素距离对应的真实距离
    double Pixel_Pitch = 0;

   

};