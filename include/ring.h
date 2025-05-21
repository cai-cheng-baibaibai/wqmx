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
	uint16_t counter = 0; // 环岛检测计数器;
    bool counter_flag = false;


    Ring();

    bool Entering_flag = 0;
    Counter entering_Left_step;
    Counter entering_Left;
    Counter outtype_Left;
    Counter Exiting_Left;
    Counter Finish_Left;

    /**
   * @brief 环岛识别初始化|复位
   *
   */
    void reset(void);

    bool process(Tracking& track, PerspectiveMapping& mapping);

    void handleNoneState(PerspectiveMapping& mapping);

private:
    /**
   * @brief 环岛类型
   *
   */
    enum RingType {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    /**
     * @brief 环岛运行步骤/阶段
     *
     */
    enum RingStep {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
        Exiting,  // 出环
        Finish    // 环任务结束
    };
    RingType ringType = RingType::RingLeft; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    bool direction;
};