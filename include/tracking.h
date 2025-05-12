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

    vector<Point> PointsEdgeLeft_BA;     // ���������ߵ㼯
    vector<Point> PointsEdgeRight_BA;    // ���������ߵ㼯
    vector<Point> yuan_PointsEdgeLeft_BA;     // ���������ߵ㼯
    vector<Point> yuan_PointsEdgeRight_BA;    // ���������ߵ㼯

    //vector<Point> PointsEdgeLeft;     // �������Ե�㼯
    //vector<Point> PointsEdgeRight;    // �����ұ�Ե�㼯
    vector<Point> midline;
      
    struct StartingPoints {
        Point left;
        Point right;
    };

    enum ImageType
    {
        Rgb = 0, // ��ֵ��Binary
        Binary,        // RGB
    };

    int rowStart = 230;

    Mat m_H;

    StartingPoints startingPoints;
    StartingPoints last_startingPoints;

    /**
    * @brief  �жϸ����ĵ��Ƿ�Ϊͼ���Ե��
    *
    * @param Point ��
    * 
    * @return 1Ϊ�߽磬0�Ǳ߽�
    */
    bool isEdgePoint(Point& Point);
    
    /**
    * @brief ������ʶ��
    *
    * 
    */
    void trackRecognition();

    /**
    * @brief ����ͼƬ����
    *
    * @param image ����ͼƬ
    */
    void trackRecognition(Mat& image, int rowStart, Mat m_h, double PixelPitch);
   
    /**
    * @brief ��ʼ��Ѱ��
    * @param rowStart ��Ե������ʼ��
    * @note ��
    */
    void Searching_for_startingPoints();

    /**
   * @brief ��ʾ������ʶ����
   *
   * @param trackImage ��Ҫ������ʾ��ͼ��
   */
    void drawImage(Mat& trackImage, double angle, Scene scene);

    /**
    * @brief ��ʼ��Ѱ��
    * @param Point �����
    * @note �жϵ��Ƿ�ɿ�
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


    ImageType imageType = ImageType::Binary; // ����ʶ������ͼ�����ͣ�RGBͼ��

    const int seeds_l[17][2] = { {1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, {0,1},{1, 1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, {0,1},{1, 1},{1,0} };
    //{-1,-1},{-1,0},{-1,+1},
    //{ 0,-1},	     {0, +1},
    //{+1,-1},{+1,0},{+1,+1},
    //�����˳ʱ��
    const int seeds_r[17][2] = { {1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0} };
    //{-1,-1},{-1,0},{-1,+1},
    //{0, -1},	     {0, +1},
    //{+1,-1},{+1,0},{+1,+1},
    //�������ʱ��

    double pix = 0;

    int rowCutUp = 10;

};