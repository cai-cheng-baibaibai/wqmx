#pragma once
#include <iostream>
struct PredictResult
{
    int type;          // ID
    std::string label; // 标签
    float score;       // 置信度
    int x;             // 坐标
    int y;             // 坐标
    int width;         // 尺寸
    int height;        // 尺寸
};