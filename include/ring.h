#include "common.h"
#include "mapping.h"
#include "tracking.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
class Ring
{
public:
	uint16_t counter = 0; // ������������;
    bool counter_flag = false;


    Ring();

    bool Entering_flag = 0;
    Counter entering_Left_step;
    Counter entering_Left;
    Counter outtype_Left;
    Counter Exiting_Left;
    Counter Finish_Left;

    /**
   * @brief ����ʶ���ʼ��|��λ
   *
   */
    void reset(void);

    bool process(Tracking& track, PerspectiveMapping& mapping);

    void handleNoneState(PerspectiveMapping& mapping);

private:
    /**
   * @brief ��������
   *
   */
    enum RingType {
        RingNone = 0, // δ֪����
        RingLeft,     // ���뻷��
        RingRight     // ���뻷��
    };

    /**
     * @brief �������в���/�׶�
     *
     */
    enum RingStep {
        None = 0, // δ֪����
        Entering, // �뻷
        Inside,   // ����
        Exiting,  // ����
        Finish    // ���������
    };
    RingType ringType = RingType::RingLeft; // ��������
    RingStep ringStep = RingStep::None;     // ��������׶�
    bool direction;
};