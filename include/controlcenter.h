#pragma once
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "common.h"
#include "mapping.h"
#include "tracking.h"
using namespace cv;
using namespace std;

#define Point_D  2

class ControlCenterCal
{
public:
	double sigmaCenter = 0;      // ���ĵ㼯�ķ���
	vector<Point> centerEdge;    // �������ĵ㼯


	//vector<Point> PointsEdgeLeft;     // �������Ե�㼯
	//vector<Point> PointsEdgeRight;    // �����ұ�Ե�㼯


	/**
  * @brief  �߽紦��
  *
  * @param �߽�
  *
  */
	void Edgehandling(Tracking& tracking, PerspectiveMapping& mapping);


	/**
  * @brief  ��ͼ
  *
  * @param trackImage ����
  *
  */
	void drawImage(Mat& trackImage);


	/**
  * @brief  ������ȡ
  *
  * @param 
  *
  */
	void midlineextraction(PerspectiveMapping& mapping,bool left);

	/**
* @brief  �����ٿ���
*
* @param
*
*/
	double pure_pursuit(Tracking& tracking, PerspectiveMapping& mapping);
	
	/**
* @brief  ���ú�������������Ƕ�
*
* @param
*
*/
	double Feedback_extraction(Tracking& tracking, PerspectiveMapping& mapping, bool dir);

	std::vector<Point> smoothTrajectory(const std::vector<cv::Point>& trajectory, int window_size = 5, double sigma = 1.0);

	void Deviation_calculation();

	void midline_track(Tracking& tracking,PerspectiveMapping& mapping);

private:
	

};