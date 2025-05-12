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
	double sigmaCenter = 0;      // 中心点集的方差
	vector<Point> centerEdge;    // 赛道中心点集


	//vector<Point> PointsEdgeLeft;     // 赛道左边缘点集
	//vector<Point> PointsEdgeRight;    // 赛道右边缘点集


	/**
  * @brief  边界处理
  *
  * @param 边界
  *
  */
	void Edgehandling(Tracking& tracking, PerspectiveMapping& mapping);


	/**
  * @brief  绘图
  *
  * @param trackImage 画布
  *
  */
	void drawImage(Mat& trackImage);


	/**
  * @brief  中线提取
  *
  * @param 
  *
  */
	void midlineextraction(PerspectiveMapping& mapping,bool left);

	/**
* @brief  纯跟踪控制
*
* @param
*
*/
	double pure_pursuit(Tracking& tracking, PerspectiveMapping& mapping);
	
	/**
* @brief  调用函数计算出反馈角度
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