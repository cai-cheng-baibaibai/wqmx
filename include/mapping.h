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
      

        
      vector<Point> PointsEdgeLeft;     // �������Ե�㼯
      vector<Point> PointsEdgeRight;    // �����ұ�Ե�㼯
      vector<Point> testpoints;     // �������Ե�㼯
      JiaoDian Left_jiaodian;
      JiaoDian Right_jiaodian;

      //0Ϊ�ұߣ�Ĭ��Ϊ�ұ�
      bool direction = RIGHT;

      PerspectiveMapping();


      double PixelPitch();

      Point Cornerdetection(const Point point);
       /**
    * @brief  ��vector͸�ӱ任�󷵻�
    *
    * @param vector
    *
    * @return 
    */
      vector<Point> Cornerdetection(const vector<Point> Points);

      Point InverseCornerDetection(const cv::Point& transformedPoint);


        /**
    * @brief  �ҽǵ�
    *
    * @param vector
    *
    * @return �ǵ㷵��
    */
      void findPointsjiaodian();

      void drawImage(Mat& trackImage);

      void Edgehandling(Tracking& tracking);

      void findSingleSideJiaodian(const vector<Point>& edgePoints,JiaoDian& output,bool isLeft,int windowSize,double minAngleDeg);

      Mat get_m_H();

      Mat m_H_inv;

private:

    //͸�ӱ任����
    Mat m_H = (cv::Mat_<double>(3, 3) <<
        2.812499999999994, 12.53906249999997, 83.75000000000071,
    -1.740597407439278e-15, 19.68749999999996, 308.7500000000004,
    -3.141197189824225e-18, 0.02343749999999995, 1);
    

    //ͼƬ���ؾ����Ӧ����ʵ����
    double Pixel_Pitch = 0;

   

};