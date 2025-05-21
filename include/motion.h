#include <fstream>
#include <iostream>
#include <cmath>
#include "common.h"
//#include "include/json.hpp"
#include "controlcenter.h"

using namespace std;
using namespace cv;

#ifndef motion_h
#define motion_h

class Motion
{
private:
    int countShift = 0; // 变速计数器
public:
    
   

    struct Params
    {
        int speedLow = 22;                           // 智能车最低速
        int speedHigh = 27;                          // 智能车最高速
        float speedBridge = 0.6;                        // 坡道速度
        float speedDown = 0.5;                          // 特殊区域降速速度
        float runP1 = 0.9;                              // 一阶比例系数：直线控制量
        float runP2 = 0.018;                            // 二阶比例系数：弯道控制量
        float runP3 = 0.0;                              // 三阶比例系数：弯道控制量
        float turnP = 3.5;                              // 一阶比例系数：转弯控制量
        float turnD = 3.5;                              // 一阶微分系数：转弯控制量
        bool debug = false;                             // 调试模式使能
        bool saveImg = false;                           // 存图使能
        uint16_t rowCutUp = 10;                         // 图像顶部切行
        uint16_t rowCutBottom = 10;                     // 图像顶部切行
        bool bridge = true;         // 坡道区使能
        bool catering = true;       // 快餐店使能
        bool layby = true;          // 临时停车区使能
        bool obstacle = true;       // 障碍区使能
        bool parking = true;        // 停车场使能
        bool ring = true;           // 环岛使能
        bool cross = true;          // 十字道路使能
        bool stop = true;           // 停车区使能

        float score = 0.5;                              // AI检测置信度
        string model = "res/model/ssd_mobilenet_v1"; // 模型路径
       // string video = "../res/samples/demo.mp4";       // 视频路径
        //NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedBridge, speedDown, runP1, runP2, runP3,
        //    turnP, turnD, debug, saveImg, rowCutUp, rowCutBottom, bridge, danger,
        //    rescue, racing, parking, ring, cross, score, model, video); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
    int speed = 20;               // 发送给电机的速度

    

    /**
     * @brief 姿态PD控制器
     *
     * @param controlCenter 智能车控制中心
     */
    void poseCtrl(int controlCenter);

    /**
     * @brief 变加速控制
     *
     * @param enable 加速使能
     * @param control
     */
    void speedCtrl(bool enable, bool slowDown, ControlCenterCal control);
  


};




#endif // !motion_h
