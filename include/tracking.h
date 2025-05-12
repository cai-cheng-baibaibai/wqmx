#pragma once
#include<iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include "common.h"

using namespace cv;
using namespace std;

#define kernel 2    

class Tracking
{
public:

    Mat Target_Image;

    vector<Point> PointsEdgeLeft_BA;     // 赛道左爬线点集
    vector<Point> PointsEdgeRight_BA;    // 赛道右爬线点集
    vector<Point> yuan_PointsEdgeLeft_BA;     // 赛道左爬线点集
    vector<Point> yuan_PointsEdgeRight_BA;    // 赛道右爬线点集

    //vector<Point> PointsEdgeLeft;     // 赛道左边缘点集
    //vector<Point> PointsEdgeRight;    // 赛道右边缘点集
    vector<Point> midline;
      
    struct StartingPoints {
        Point left;
        Point right;
    };

    enum ImageType
    {
        Rgb = 0, // 二值化Binary
        Binary,        // RGB
    };

    int rowStart = 230;

    Mat m_H;

    StartingPoints startingPoints;
    StartingPoints last_startingPoints;

    /**
    * @brief  判断给定的点是否为图像边缘点
    *
    * @param Point 点
    * 
    * @return 1为边界，0非边界
    */
    bool isEdgePoint(Point& Point);
    
    /**
    * @brief 赛道线识别
    *
    * 
    */
    void trackRecognition();

    /**
    * @brief 赛道图片导入
    *
    * @param image 赛道图片
    */
    void trackRecognition(Mat& image, int rowStart, Mat m_h, double PixelPitch);
   
    /**
    * @brief 起始点寻找
    * @param rowStart 边缘搜索起始行
    * @note 无
    */
    void Searching_for_startingPoints();

    /**
   * @brief 显示赛道线识别结果
   *
   * @param trackImage 需要叠加显示的图像
   */
    void drawImage(Mat& trackImage, double angle, Scene scene);

    /**
    * @brief 起始点寻找
    * @param Point 输入点
    * @note 判断点是否可靠
    */
    bool isStartingPoints(Point Point, bool left);

    std::vector<cv::Point> smoothTrajectory(const std::vector<cv::Point>& trajectory, int window_size, double sigma);


    void Edgecleaning(void);

    int re_rowstart(void);

    void PointsEdge_extract(void);

    bool yuanxian = 0;

    void searching_yuanxian();

private:

    StartingPoints yuan_startingPoints;


    ImageType imageType = ImageType::Binary; // 赛道识别输入图像类型：RGB图像

    const int seeds_l[17][2] = { {1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, {0,1},{1, 1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, {0,1},{1, 1},{1,0} };
    //{-1,-1},{-1,0},{-1,+1},
    //{ 0,-1},	     {0, +1},
    //{+1,-1},{+1,0},{+1,+1},
    //这个是顺时针
    const int seeds_r[17][2] = { {1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0} };
    //{-1,-1},{-1,0},{-1,+1},
    //{0, -1},	     {0, +1},
    //{+1,-1},{+1,0},{+1,+1},
    //这个是逆时针

    double pix = 0;

    int rowCutUp = 10;

};