#pragma once
#include <iostream>
struct PredictResult
{
    int type;          // ID
    std::string label; // ��ǩ
    float score;       // ���Ŷ�
    int x;             // ����
    int y;             // ����
    int width;         // �ߴ�
    int height;        // �ߴ�
};