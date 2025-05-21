#pragma once
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include <chrono>
#define COLSIMAGE 320    // ͼ�������
#define ROWSIMAGE 240    // ͼ�������
#define PWMSERVOMAX 1900 // ���PWM���ֵ����1840
#define PWMSERVOMID 1500 // ���PWM��ֵ 1520
#define PWMSERVOMIN 1100 // ���PWM��Сֵ���ң�1200
#define void_row 170
#define void_col_min 114
#define void_col_max 210


using namespace cv;
using namespace std;

enum Scene
{
    NormalScene = 0, // ��������
    CrossScene,      // ʮ�ֵ�·
    RingScene,       // ������·
    BridgeScene,     // �µ���
    ObstacleScene,   // �ϰ���
    CateringScene,   // ��͵�
    LaybyScene,      // ��ʱͣ����
    ParkingScene,    // ͣ����
    StopScene        // ͣ����������
};


#define LABEL_BATTERY 0    // AI��ǩ�����վ
#define LABEL_BLOCK 1      // AI��ǩ���ϰ���
#define LABEL_BRIDGE 2     // AI��ǩ���µ�
#define LABEL_BURGER 3     // AI��ǩ������
#define LABEL_CAR 4        // AI��ǩ�����߳�
#define LABEL_COMPANY 5    // AI��ǩ����˾
#define LABEL_CONE 6       // AI��ǩ��׶Ͱ
#define LABEL_CROSSWALK 7  // AI��ǩ��������
#define LABEL_PEDESTRIAN 8 // AI��ǩ������
#define LABEL_SCHOOL 9     // AI��ǩ��ѧУ

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
        count = std::min(count + 1, threshold); // ��ֹ���
    }

    void checkAndReset() {
        if (count >= threshold) {
            action();
        }
        count = 0;
    }
};

// �������������ļнǣ����ػ����ƣ�
double calculateAngle(const Point& a, const Point& b);

bool isStraightLine(const std::vector<cv::Point>& points, double max_error = 5.0);


double Orthogonalvector(Point& a, Point& b);

Point Cornerdetection(const Point point, Mat m_H);

// ֡�ʲ���
void framerate();

//����
double sigma(vector<Point> vec);

string img_path(string path);