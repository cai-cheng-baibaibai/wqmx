#include "../../include/crossroad.h"

void Crossroad::reset(void)
{
    crossroadSype = CrossroadSype::None; // 十字道路类型
}

Crossroad::Crossroad()
{
    cross_Begin.threshold = 8;
    cross_Begin.action = [this] {
        crossroadSype = CrossroadSype::Cross_Begin;
        };
    cross_IN.threshold = 8;
    cross_IN.action = [this] {
        crossroadSype = CrossroadSype::Cross_IN;
        };
    none.threshold = 8;
    none.action = [this] {
        crossroadSype = CrossroadSype::None;
        pt_Left = Point(0, 0);
        pt_Right = Point(0, 0);
        cross_IN_flag = 0;
        rowStart = 50;
        };
    outtype.threshold = 8;
    outtype.action = [this]() {
        crossroadSype = CrossroadSype::None;
        pt_Left = Point(0, 0);
        pt_Right = Point(0, 0);
        cross_IN_flag = 0;
        rowStart = 50;
        };
    cross_IN_step.threshold = 8;
    cross_IN_step.action = [this]() {
        cross_IN_flag = 1;
        };
    cross_OUT.threshold = 8;
    cross_OUT.action = [this]() {
        crossroadSype = CrossroadSype::Cross_OUT;
        rowStart = first_rowStart;
        cross_IN_flag = 0;
        };

}

bool Crossroad::crossRecognition(Tracking& track, PerspectiveMapping& mapping)
{
    static int counter = 0;
    static bool counter_flag = 0;

    if (!first_rowStart)
        first_rowStart = track.rowStart;
    if (crossroadSype == CrossroadSype::None)
    {
        track.rowStart = first_rowStart;
    }

    /*if (crossroadSype != CrossroadSype::None && crossroadSype != CrossroadSype::Cross_IN)
    {
        if (isStraightLine(mapping.PointsEdgeLeft) == true && isStraightLine(mapping.PointsEdgeRight) == true)
        {
            counter_flag = 1;
            outtype.increment();
        }
    }*/

    if (crossroadSype == CrossroadSype::None       &&
        mapping.Left_jiaodian.angle < 115 && mapping.Right_jiaodian.angle < 115 )
    {
        Point left_jiaodian = mapping.InverseCornerDetection(mapping.Left_jiaodian.jiaodian);
        Point right_jiaodian = mapping.InverseCornerDetection(mapping.Right_jiaodian.jiaodian);
        int dx = mapping.Left_jiaodian.jiaodian.x - mapping.Right_jiaodian.jiaodian.x;
        int dy = mapping.Left_jiaodian.jiaodian.y - mapping.Right_jiaodian.jiaodian.y;
        double dis = sqrt(dx * dx + dy * dy) * mapping.PixelPitch();

        double left_angle = (left_jiaodian.x - track.PointsEdgeLeft_BA.back().x) / (left_jiaodian.y - track.PointsEdgeLeft_BA.back().y + 1e-5);
        double right_angle = (right_jiaodian.x - track.PointsEdgeRight_BA.back().x) / (right_jiaodian.y - track.PointsEdgeRight_BA.back().y + 1e-5);

        if (left_jiaodian.y < right_jiaodian.y &&  dis > 40 && dis < 65  && abs(left_angle - right_angle) < 10 && left_jiaodian.x > ROWSIMAGE / 5 && right_jiaodian.x > ROWSIMAGE / 5)
        {
            counter_flag = 1;
            cross_Begin.increment();
            pt_Left = mapping.Left_jiaodian.jiaodian;
            pt_Right = mapping.Right_jiaodian.jiaodian;
        }
    }
    if (crossroadSype == CrossroadSype::Cross_Begin)
    {
        Point left_jiaodian = mapping.Left_jiaodian.jiaodian;
        Point right_jiaodian = mapping.Right_jiaodian.jiaodian;
        int dx = left_jiaodian.x - pt_Left.x;
        int dy = left_jiaodian.y - pt_Left.y;
        double dis = sqrt(dx * dx + dy * dy) * mapping.PixelPitch();
        if (dis < 50 && mapping.Left_jiaodian.angle < 110 && !track.PointsEdgeRight_BA.empty())
        {
            pt_Left = left_jiaodian;
        }
        dx = right_jiaodian.x - pt_Right.x;
        dy = right_jiaodian.y - pt_Right.y;
        dis = sqrt(dx * dx + dy * dy) * mapping.PixelPitch();
        if (dis < 50 && mapping.Right_jiaodian.angle < 110 && !track.PointsEdgeLeft_BA.empty())
        {
            pt_Right = right_jiaodian;
        }
        if (mapping.InverseCornerDetection(pt_Left).x > ROWSIMAGE / 2 || mapping.InverseCornerDetection(pt_Right).x > ROWSIMAGE / 2)
        {
            dx = pt_Right.x - pt_Left.x;
            dy = pt_Right.y - pt_Left.y;
            double slope = -1.0 / (dx / (dy + 1e-5));
            int len = 40 / 2;
            Point start = pt_Right;

            mapping.PointsEdgeRight.clear();


            for (int y = len; y >= -len; y--) {
                int x = (start.x + slope * y); // 根据斜率计算 y 值
                Point pt;
                pt.x = x;
                pt.y = y + start.y;
                mapping.PointsEdgeRight.push_back(pt); // 将 POINT 存入点集
            }
        }
        
        if (track.PointsEdgeLeft_BA.empty() && track.PointsEdgeRight_BA.empty())
        {
            counter_flag = 1;
            cross_IN.increment();
        }

    }

    if (crossroadSype == CrossroadSype::Cross_IN)
    {
        
        if (track.PointsEdgeLeft_BA.size() > 50)
        {
            int dx = track.PointsEdgeLeft_BA.front().x - track.PointsEdgeLeft_BA.back().x;
            int dy = track.PointsEdgeLeft_BA.front().y - track.PointsEdgeLeft_BA.back().y;
            double dis = sqrt(dx * dx + dy * dy);
            if (dis < 20)
            {
                int max_x = track.PointsEdgeLeft_BA[0].x;
                for (int i = 0; i < track.PointsEdgeLeft_BA.size(); i++)
                {
                    if (track.PointsEdgeLeft_BA[i].x > max_x)
                    {
                        max_x = track.PointsEdgeLeft_BA[i].x;
                    }
                }
                rowStart = max_x - 10;
                if (rowStart > ROWSIMAGE / 2)
                {
                    rowStart = ROWSIMAGE / 2;
                }
            }
        }
        
        if (track.PointsEdgeRight_BA.size() > 50)
        {
            int dx = track.PointsEdgeRight_BA.front().x - track.PointsEdgeRight_BA.back().x;
            int dy = track.PointsEdgeRight_BA.front().y - track.PointsEdgeRight_BA.back().y;
            double dis = sqrt(dx * dx + dy * dy);
            if (dis < 20)
            {
                int max_x = track.PointsEdgeRight_BA[0].x;
                for (int i = 0; i < track.PointsEdgeRight_BA.size(); i++)
                {
                    if (track.PointsEdgeRight_BA[i].x > max_x)
                    {
                        max_x = track.PointsEdgeRight_BA[i].x;
                    }
                }
                rowStart = max_x - 10;
                if (rowStart > ROWSIMAGE / 2)
                {
                    rowStart = ROWSIMAGE / 2;
                }
            }
        }

        //进入完成判断
        track.startingPoints.left = Point(0, 0);
        track.startingPoints.right = Point(0, 0);
        track.rowStart = first_rowStart;
        track.Searching_for_startingPoints();
        if (track.startingPoints.left.y == 0 && track.startingPoints.right.y == 0)
        {
            counter_flag = 1;
            cross_IN_step.increment();
        }
        if (cross_IN_flag && track.startingPoints.left.y != 0 && track.startingPoints.right.y != 0)
        {
            counter_flag = 1;
            cross_OUT.increment();
            //none.increment();
        }

        track.rowStart = rowStart;
    }

    else if (crossroadSype == CrossroadSype::Cross_OUT)
    {
        if (!track.PointsEdgeRight_BA.empty())
        {
            mapping.direction = RIGHT;
        }
        else
        {
            mapping.direction = LEFT;
        }

        if (!track.PointsEdgeLeft_BA.empty() && !track.PointsEdgeRight_BA.empty())
        {
            if(track.PointsEdgeLeft_BA.back().y < 10 && track.PointsEdgeLeft_BA.back().x > ROWSIMAGE / 2
                && track.PointsEdgeRight_BA.back().y > COLSIMAGE - 10 && track.PointsEdgeRight_BA.back().x > ROWSIMAGE / 2)
            {
                rowStart = 40;
            }
        }

        //进入完成判断
        track.startingPoints.left = Point(0, 0);
        track.startingPoints.right = Point(0, 0);
        track.rowStart = first_rowStart;
        track.Searching_for_startingPoints();
        if (track.startingPoints.left.y == 0 && track.startingPoints.right.y == 0)
        {
            counter_flag = 1;
            cross_IN_step.increment();
        }
        if (cross_IN_flag && track.startingPoints.left.y != 0 && track.startingPoints.right.y != 0)
        {
            counter_flag = 1;
            //cross_OUT.increment();
            none.increment();
        }

        track.rowStart = rowStart;
    }
   
    if (counter_flag)
    {
        if (++counter >= 15)
        {
            cross_Begin.checkAndReset();
            cross_IN.checkAndReset();
            none.checkAndReset();
            outtype.checkAndReset();
            cross_IN_step.checkAndReset();
            cross_OUT.checkAndReset();
            counter = 0;
            counter_flag = 0;
        }
       
    }
    return crossroadSype != CrossroadSype::None;
}