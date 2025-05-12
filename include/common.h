#pragma once
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include <chrono>
#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
#define PWMSERVOMAX 1900 // 舵机PWM最大值（左）1840
#define PWMSERVOMID 1500 // 舵机PWM中值 1520
#define PWMSERVOMIN 1100 // 舵机PWM最小值（右）1200
#define void_row 170
#define void_col_min 114
#define void_col_max 210


using namespace cv;
using namespace std;

enum Scene
{
    NormalScene = 0, // 基础赛道
    CrossScene,      // 十字道路
    RingScene,       // 环岛道路
    BridgeScene,     // 坡道区
    ObstacleScene,   // 障碍区
    CateringScene,   // 快餐店
    LaybyScene,      // 临时停车区
    ParkingScene,    // 停车区
    StopScene        // 停车（结束）
};

struct POINT
{
    int x = 0;
    int y = 0;
    float slope = 0.0f;

    POINT() {};
    POINT(int x, int y) : x(x), y(y) {};
};

struct Counter {
    int count = 0;
    int threshold;
    std::function<void()> action;

    void increment() {
        count = std::min(count + 1, threshold); // 防止溢出
    }

    void checkAndReset() {
        if (count >= threshold) {
            action();
        }
        count = 0;
    }
};

// 计算两个向量的夹角（返回弧度制）
double calculateAngle(const Point& a, const Point& b);

bool isStraightLine(const std::vector<cv::Point>& points, double max_error = 5.0);


double Orthogonalvector(Point& a, Point& b);

Point Cornerdetection(const Point point, Mat m_H);

// 帧率测量
void framerate();

//方差
double sigma(vector<Point> vec);

string img_path(string path);