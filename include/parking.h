#pragma once
 #include <fstream>
 #include <iostream>
 #include <cmath>
 #include <opencv2/highgui.hpp>
 #include <opencv2/opencv.hpp>
 #include "common.h"
 #include "tracking.h"
 #include  "mapping.h"

using namespace cv;
using namespace std;

class Parking
{
public:

    /**
     * @brief 停车步骤
     *
     */
    enum ParkStep
    {
        none = 0, // 未知状态
        enable,   // 停车场使能
        turning,  // 入库转向
        stop,     // 停车
        trackout,  // 出库
        out
    };

    enum ParkType {
        ParkLeft,     
        ParkRight     
    };
    
    ParkStep step = ParkStep::none; // 停车步骤
    ParkType Parkstep = ParkType::ParkRight;

    bool process(Tracking& track, Mat& image, PerspectiveMapping& mapping);
    

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(Tracking track, Mat& image);

private:
    Vec4i front_line = 0;
    Vec4i mid_line = 0;
    Vec4i end_line = 0;
    


    uint16_t counterSession = 0;  // 图像场次计数器
    uint16_t counterRec = 0;      // 加油站标志检测计数器
    bool garageFirst = true;      // 进入一号车库
    int lineY = 0;                // 直线高度
    int lineX = 0;                // 直线高度
    bool startTurning = false;    // 开始转弯
    vector<vector<Point>> pathsEdgeLeft; // 记录入库路径
    vector<vector<Point>> pathsEdgeRight;
    Point ptA = Point(0, 0);      // 记录线段的两个端点和角度
    Point ptB = Point(0, 0);
    double pt_angle = 0;
    int truningTime = 21;             // 转弯时间 21帧
    int stopTime = 30;                // 停车时间 40帧
    float swerveTime = 0.4;           // 转向时机 0.2 （转弯线出现在屏幕上方0.2处）
};