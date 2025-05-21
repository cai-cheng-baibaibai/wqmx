#include "../include/motion.h"

using namespace std;
using namespace cv;



void Motion::poseCtrl(int controlCenter)
{
    float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
    static int errorLast = 0;                    // 记录前一次的偏差
    if (abs(error - errorLast) > COLSIMAGE / 10)
    {
        error = error > errorLast ? errorLast + COLSIMAGE / 10 : errorLast - COLSIMAGE / 10;
    }

    params.turnP = abs(error) * params.runP2 + params.runP1;
    int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
    errorLast = error;

    servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
}

void Motion::speedCtrl(bool enable, bool slowDown, ControlCenterCal control)
{
    // 控制率
    uint8_t controlLow = 0;   // 速度控制下限
    uint8_t controlMid = 5;   // 控制率
    uint8_t controlHigh = 10; // 速度控制上限

    if (slowDown)
    {
        countShift = controlLow;
        speed = params.speedDown;
    }
    else if (enable) // 加速使能
    {
        if (control.centerEdge.size() < 10)
        {
            speed = params.speedLow;
            countShift = controlLow;
            return;
        }
        if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2)
        {
            speed = params.speedLow;
            countShift = controlLow;
            return;
        }
        if (abs(control.sigmaCenter) < 100.0)
        {
            countShift++;
            if (countShift > controlHigh)
                countShift = controlHigh;
        }
        else
        {
            countShift--;
            if (countShift < controlLow)
                countShift = controlLow;
        }

        if (countShift > controlMid)
            speed = params.speedHigh;
        else
            speed = params.speedLow;
    }
    else
    {
        countShift = controlLow;
        speed = params.speedLow;
    }
}