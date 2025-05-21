#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "common.h"
#include "tracking.h"
#include "common.h"
#include "mapping.h"
#include "detection.hpp" // Ai模型预测

using namespace std;
using namespace cv;

class Obstacle
{

public:
    
    bool process(Tracking& track, vector<PredictResult> predict, PerspectiveMapping& mapping);
    
    /**
    * @brief 图像绘制禁行区识别结果
    *
    * @param img 需要叠加显示的图像
    */
    void drawImage(Mat& img);

private:
    bool enable = false;     // 场景检测使能标志
   
    PredictResult resultObs;

   
};
