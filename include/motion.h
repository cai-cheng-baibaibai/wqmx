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
    int countShift = 0; // ���ټ�����
public:
    
   

    struct Params
    {
        int speedLow = 22;                           // ���ܳ������
        int speedHigh = 27;                          // ���ܳ������
        float speedBridge = 0.6;                        // �µ��ٶ�
        float speedDown = 0.5;                          // �����������ٶ�
        float runP1 = 0.9;                              // һ�ױ���ϵ����ֱ�߿�����
        float runP2 = 0.018;                            // ���ױ���ϵ�������������
        float runP3 = 0.0;                              // ���ױ���ϵ�������������
        float turnP = 3.5;                              // һ�ױ���ϵ����ת�������
        float turnD = 3.5;                              // һ��΢��ϵ����ת�������
        bool debug = false;                             // ����ģʽʹ��
        bool saveImg = false;                           // ��ͼʹ��
        uint16_t rowCutUp = 10;                         // ͼ�񶥲�����
        uint16_t rowCutBottom = 10;                     // ͼ�񶥲�����
        bool bridge = true;         // �µ���ʹ��
        bool catering = true;       // ��͵�ʹ��
        bool layby = true;          // ��ʱͣ����ʹ��
        bool obstacle = true;       // �ϰ���ʹ��
        bool parking = true;        // ͣ����ʹ��
        bool ring = true;           // ����ʹ��
        bool cross = true;          // ʮ�ֵ�·ʹ��
        bool stop = true;           // ͣ����ʹ��

        float score = 0.5;                              // AI������Ŷ�
        string model = "res/model/ssd_mobilenet_v1"; // ģ��·��
       // string video = "../res/samples/demo.mp4";       // ��Ƶ·��
        //NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedBridge, speedDown, runP1, runP2, runP3,
        //    turnP, turnD, debug, saveImg, rowCutUp, rowCutBottom, bridge, danger,
        //    rescue, racing, parking, ring, cross, score, model, video); // ��ӹ��캯��
    };

    Params params;                   // ��ȡ���Ʋ���
    uint16_t servoPwm = PWMSERVOMID; // ���͸������PWM
    int speed = 20;               // ���͸�������ٶ�

    

    /**
     * @brief ��̬PD������
     *
     * @param controlCenter ���ܳ���������
     */
    void poseCtrl(int controlCenter);

    /**
     * @brief ����ٿ���
     *
     * @param enable ����ʹ��
     * @param control
     */
    void speedCtrl(bool enable, bool slowDown, ControlCenterCal control);
  


};




#endif // !motion_h
