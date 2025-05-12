#include "../../include/ring.h"

void Ring::reset(void) {
    ringType = RingType::RingLeft; // 环岛类型
    ringStep = RingStep::None;     // 环岛处理阶段
}

Ring::Ring() {  // 构造函数
    entering_Left.threshold = 13;
    entering_Left.action = [this] {
        ringType = RingType::RingLeft;       
        ringStep = RingStep::Entering;
       };
    entering_Left_step.threshold = 8;
    entering_Left_step.action = [this] {
        Entering_flag = 1;
        };
    outtype_Left.threshold = 8;
    outtype_Left.action = [this]() {
        ringType = RingType::RingLeft;
        ringStep = RingStep::None;
        };

    Exiting_Left.threshold = 8;
    Exiting_Left.action = [this]() {
        ringType = RingType::RingLeft;
        ringStep = RingStep::Exiting;
        };

    Finish_Left.threshold = 8;
    Finish_Left.action = [this]() {
        ringType = RingType::RingLeft;
        ringStep = RingStep::None;
        };

}

bool Ring::process(Tracking& track, PerspectiveMapping& mapping)
{
    if (ringStep == RingStep::None)
    {
        Point left_jiaodian = mapping.InverseCornerDetection(mapping.Left_jiaodian.jiaodian);
        if (mapping.Left_jiaodian.angle < 100 && left_jiaodian.y < COLSIMAGE / 2 && left_jiaodian.x > ROWSIMAGE / 3)
        {
            if (isStraightLine(mapping.PointsEdgeRight) == true && isStraightLine(mapping.PointsEdgeLeft) == false
                )
            {
                counter_flag = 1;
                entering_Left.increment();
            }
        }
    }

    if (ringType == RingType::RingLeft && ringStep == RingStep::Entering)
    {
        //mapping.direction = 1;
        if (mapping.PointsEdgeLeft.empty())
        {
            counter_flag = 1;
            entering_Left_step.increment();
        }
        if (Entering_flag == 1 && !mapping.PointsEdgeLeft.empty())
        {
            Entering_flag = 0;
            mapping.direction = 1;
            ringStep = RingStep::Inside;
        }
        //if (!track.PointsEdgeRight_BA.empty())
        //{
        //  
        //        ringStep = RingStep::Inside;
        //       // mapping.direction = 0;

        //}
    }


    if (ringType == RingType::RingLeft && ringStep == RingStep::Inside)
    {
        mapping.direction = 1;
        if (mapping.Right_jiaodian.angle < 110)
        {
            counter_flag = 1;
            Exiting_Left.increment();
        }
    }

    if (ringType == RingType::RingLeft && ringStep == RingStep::Exiting)
    {
        mapping.direction = 1;
        if (isStraightLine(mapping.PointsEdgeRight) == true)
        {
            counter_flag = 1;
            Finish_Left.increment();
        }
    }

    if (ringType == RingType::RingLeft && ringStep != RingStep::None)
    {
        if (isStraightLine(mapping.PointsEdgeLeft) == true)
        {
            counter_flag = 1;
            outtype_Left.increment();
        }
    }



    if (counter_flag)
    {
        if (++counter >= 15)
        {
            entering_Left_step.checkAndReset();
            entering_Left.checkAndReset();
            outtype_Left.checkAndReset();
            Exiting_Left.checkAndReset();
            Finish_Left.checkAndReset();
            counter = 0;
            counter_flag = 0;
        }
    }

    return ringStep != RingStep::None;
}