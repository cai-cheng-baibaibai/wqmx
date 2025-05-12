#include "../../include/parking.h"


bool Parking::process(Tracking& track, Mat& image, PerspectiveMapping& mapping)
{
    counterSession++;
    //if (step != ParkStep::none && counterSession > 80) // 超时退出
    //{
    //    counterRec = 0;
    //    counterSession = 0;
    //    step = ParkStep::none;   // 退出状态.
    //    startTurning = false;    // 恢复状态
    //    garageFirst = true;      // 进入一号车库
    //    lineY = 0;               // 直线高度
    //    ptA = Point(0, 0);       // 清空线段的两个端点
    //    ptB = Point(0, 0);
    //    std::cout << "退出停车场" << std::endl;
    //}
    switch (step)
    {
    case ParkStep::none: // AI未识别
    {
        //for (size_t i = 0; i < predict.size(); i++)
        //{
        //    if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.4)
        //    {
        //        counterRec++;
        //        break;
        //    }
        //}
        //if (counterRec) // 检测到一帧后开始连续监测AI标志是否满足条件
        //{
        //    if (counterRec >= 3 && counterSession < 8)
        //    {
        //        counterRec = 0;
        //        counterSession = 0;
        //        step = ParkStep::enable; // 检测到停车场标志
        //        std::cout << "进入停车场" << std::endl;
        //        return true;
        //    }
        //    else if (counterSession >= 8)
        //    {
        //        counterRec = 0;
        //        counterSession = 0;
        //    }
        //}
        return false;
        break;
    }
    case ParkStep::enable: // 停车场使能
    {
        //int carY = ROWSIMAGE;
        int carY = 0;
        int batteryY = ROWSIMAGE;     // 充电站标识高度
        if (Parkstep == ParkType::ParkRight)
        {
            mapping.direction = 1;
        }
        else
        {
            mapping.direction = 0;
        }
        //for (size_t i = 0; i < predict.size(); i++)
        //{
        //    if (predict[i].type == LABEL_CAR && predict[i].score > 0.6)
        //    {
        //        carY = (predict[i].y + predict[i].height)/2;   // 计算智能车的中心高度
        //    }
        //    else if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.6)
        //    {
        //        batteryY = predict[i].y ;   // 计算标识牌最高高度
        //    }
        //}
        // 图像预处理
        Mat edges, dstImage;
        GaussianBlur(image, dstImage, Size(5, 5), 0, 0);
        Canny(dstImage, edges, 50, 150);
        // 霍夫变换检测直线
        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 10);

        // 按中点的 y 坐标对线段进行排序
        std::sort(lines.begin(), lines.end(), [](const Vec4i& a, const Vec4i& b) {
            double midY1 = (a[1] + a[3]) / 2.0;
            double midY2 = (b[1] + b[3]) / 2.0;
            return midY1 > midY2;
            });

        vector<Vec4i> horizontalLines;
        Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        for (const Vec4i& line : lines)
        {
            Point pt1(line[0], line[1]);
            Point pt2(line[2], line[3]);

            double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
            int midX = (line[0] + line[2]) / 2;
            int midY = (line[1] + line[3]) / 2;
            // 筛选直线，直线只出现在右侧并且在充电标识牌的上方
            //if (abs(angle) < 30 && angle < 0 && midY < batteryY && midY < 200 && midX > COLSIMAGE / 2)
            //{ // 接近水平
            //    horizontalLines.push_back(line);
            //    cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
            //}
            if (abs(angle) < 30 && angle < 0  && midY < 200 && midX > COLSIMAGE / 2)
                { // 接近水平
                    horizontalLines.push_back(line);
                    cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                }
        }
       /* imshow("Detected Lines", imgRes);
        moveWindow("Detected Lines", 400, 100);
        waitKey(1);*/

        //Mat imgRes1 = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        if (horizontalLines.size() >= 2)
        {
            std::vector<cv::Vec4i> candidateLines;
            // 查找平行且距离大于20的线对
            candidateLines.push_back(horizontalLines[0]);
            for (size_t i = 1; i < horizontalLines.size(); i++) {
                int midY1 = (horizontalLines[i][1] + horizontalLines[i][3]) / 2;
                int midY2 = (candidateLines.back()[1] + candidateLines.back()[3]) / 2;

                int midX1 = (horizontalLines[i][0] + horizontalLines[i][2]) / 2;
                int midX2 = (candidateLines.back()[0] + candidateLines.back()[2]) / 2;

                Vec4i line1 = horizontalLines[i];
                Vec4i line2 = candidateLines.back();

                // 计算两条线的角度
                double angle1 = atan2(line1[3] - line1[1], line1[2] - line1[0]) * 180.0 / CV_PI;
                double angle2 = atan2(line2[3] - line2[1], line2[2] - line2[0]) * 180.0 / CV_PI;
                if (abs(midY1 - midY2) > 10 && abs(angle1 - angle2) < 15 && midX1 < midX2 && step == ParkStep::enable)
                {
                    candidateLines.push_back(horizontalLines[i]);
                }
            }
            for (int i = 0; i < candidateLines.size(); i++)
            {
                //cv::line(imgRes1, Point(candidateLines[i][0], candidateLines[i][1]), Point(candidateLines[i][2], candidateLines[i][3]), Scalar(0, 0, 255), 2);
            }

            if (candidateLines.size() == 3)
            {
                front_line = candidateLines[0];
                mid_line = candidateLines[1];
                end_line = candidateLines[2];
            }
        }

        if (mid_line[0] != 0)
        {
            int midY = (mid_line[1] + mid_line[3]) / 2;
            if (carY > midY)
            {
                garageFirst = true;         // 进入一号车库(远)
                lineY = (end_line[1] + end_line[3]) / 2;;  // 获取距离最远的线控制车入库
                lineX = (end_line[0] + end_line[2]) / 2;;  // 获取距离最远的线控制车入库
                step = ParkStep::turning; // 开始入库
                counterSession = 0;
            }
            else
            {
                garageFirst = true;         // 进入一号车库(远)
                lineY = (mid_line[1] + mid_line[3]) / 2;  // 获取距离最远的线控制车入库
                lineX = (mid_line[0] + mid_line[2]) / 2;  // 获取距离最远的线控制车入库
                step = ParkStep::turning; // 开始入库
                counterSession = 0;
            }

            break;
        }


       /* imshow("imgRes1", imgRes1);
        moveWindow("imgRes1", 700, 100);
        waitKey(1);*/

        break;
    }
    case ParkStep::turning: // 入库转向
    {
        // 图像预处理
        Mat edges;
        Canny(image, edges, 50, 150);

        // 霍夫变换检测直线
        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 10);

        //Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        for (const Vec4i& line : lines) {
            Point pt1(line[0], line[1]);
            Point pt2(line[2], line[3]);

            int midX = (line[0] + line[2]) / 2;
            int midY = (line[1] + line[3]) / 2;
            double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;

            // 接近水平并且在右侧
            if (!startTurning)
            {
                if (abs(angle) < 40 && angle < 0 && midX > COLSIMAGE / 2)
                {
                    if ( midY > lineY && (midY - lineY) <= 10) // 限制线段增加值
                    {
                        lineX = midX;
                        lineY = midY; // 更新直线高度
                        ptA = pt1;    // 更新端点
                        ptB = pt2;
                        pt_angle = angle;
                    }
                }
            }
            else
            {
                if (abs(midY - lineY) <= 30 && abs(pt_angle - angle) < 30 && abs(midX - lineX) < 30) 
                {
                    lineX = midX;
                    lineY = midY; // 更新直线高度
                    ptA = pt1;    // 更新端点
                    ptB = pt2;
                    pt_angle = angle;
                }
            }
          
        }

        /*cv::line(imgRes, ptA, ptB, Scalar(0, 0, 255), 2);
         imshow("Detected Lines", imgRes);
         waitKey(1);*/

        if (lineY > 80 || startTurning) // 控制转弯时机
        {
            if (!startTurning)
            {
                counterSession = 0;
                startTurning = true; // 已经开始转弯
            }
            std::cout << "控制转弯" << std::endl;
            // 计算直线的斜率
            double slope = static_cast<double>(ptB.y - ptA.y) / (ptB.x - ptA.x + 1e-5); // 避免除零
            int y3 = slope * (0 - ptA.x) + ptA.y;        // 延长起点的Y坐标
            int y4 = slope * (COLSIMAGE - ptA.x) + ptA.y;// 延长终点的Y坐标

            Point start(0, y3);           // 延长起点
            Point end(COLSIMAGE, y4);     // 延长终点

            track.PointsEdgeLeft_BA.clear(); // 清空原始点集

            for (int x = start.x; x <= end.x; x++) {
                int y = static_cast<int>(start.y + slope * (x - start.x)); // 根据斜率计算 y 值
                Point pt;
                pt.x = y; // 将 cv::Point 的 x 赋值给 POINT 的 y
                pt.y = x; // 将 cv::Point 的 y 赋值给 POINT 的 x
                track.PointsEdgeLeft_BA.push_back(pt); // 将 POINT 存入点集
            }
            mapping.PointsEdgeLeft.clear();
            mapping.PointsEdgeLeft = mapping.Cornerdetection(track.PointsEdgeLeft_BA);


            pathsEdgeLeft.push_back(mapping.PointsEdgeLeft); // 记录进厂轨迹
            pathsEdgeRight.push_back(mapping.PointsEdgeRight);
        }
        //if (counterSession > truningTime && startTurning) // 开始停车状态
        //{
        //    //std::cout << "开始停车" << std::endl;
        //    step = ParkStep::stop; // 开始停车
        //}
 
        //已经开始转弯,判断停止
        Mat imgRes1 = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        if (startTurning)
        {
            HoughLinesP(edges, lines, 1, CV_PI / 180, 40, 40, 20);
            for (const Vec4i& line : lines)
            {
                Point pt1(line[0], line[1]);
                Point pt2(line[2], line[3]);

                int midX = (line[0] + line[2]) / 2;
                int midY = (line[1] + line[3]) / 2;
                double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;

                if (angle > 0 && abs(angle) < 20 && midY < void_row && abs(midY - void_row) < 20 && midX < COLSIMAGE / 2)
                {
                    std::cout << "stopping" << std::endl;
                    step = ParkStep::stop; // 开始停车
                    counterSession = 0;
                }
                
            }
           /* for (int i = 0; i < lines.size(); i++)
            {
                cv::line(imgRes1, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 2);
            }*/
        }

        /*imshow("imgRes1", imgRes1);
        moveWindow("imgRes1", 400, 100);*/

        break;
    }
    case ParkStep::stop: // 停车
    {
        if (counterSession > stopTime) // 倒车状态
            step = ParkStep::trackout; // 开始倒车
        break;
    }
    case ParkStep::trackout: // 出库
    {
        //std::cout << "开始倒车" << std::endl;
        if (pathsEdgeLeft.empty() || pathsEdgeRight.empty())
        {
            counterRec = 0;
            counterSession = 0;
            step = ParkStep::out;   // 退出状态.
            startTurning = false;    // 恢复状态
            garageFirst = true;      // 进入一号车库
            lineY = 0;               // 直线高度
            lineX = 0;
            ptA = Point(0, 0);       // 清空线段的两个端点
            ptB = Point(0, 0);
            pt_angle = 0;
            pathsEdgeRight.clear();
            pathsEdgeLeft.clear();
            break;
            //std::cout << "退出停车场" << std::endl;
        }
        if (isStraightLine(mapping.PointsEdgeLeft) == true)
        {
            if (!track.PointsEdgeLeft_BA.empty() && track.PointsEdgeLeft_BA.back().x < 30)
            {
                counterRec = 0;
                counterSession = 0;
                step = ParkStep::out;   // 退出状态.
                startTurning = false;    // 恢复状态
                garageFirst = true;      // 进入一号车库
                lineY = 0;               // 直线高度
                lineX = 0;
                ptA = Point(0, 0);       // 清空线段的两个端点
                ptB = Point(0, 0);
                pt_angle = 0;
                pathsEdgeRight.clear();
                pathsEdgeLeft.clear();
                break;
            }
        }

        //if (counterSession > 40 && (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1))
        //{
        //    counterRec = 0;
        //    counterSession = 0;
        //    step = ParkStep::out;   // 退出状态.
        //    startTurning = false;    // 恢复状态
        //    garageFirst = true;      // 进入一号车库
        //    lineY = 0;               // 直线高度
        //    lineX = 0;
        //    ptA = Point(0, 0);       // 清空线段的两个端点
        //    ptB = Point(0, 0);
        //    pt_angle = 0;
        //    pathsEdgeRight.clear();
        //    pathsEdgeLeft.clear();
        //    std::cout << "退出停车场" << std::endl;
        //    break;
        //}

        mapping.PointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
        mapping.PointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
        pathsEdgeLeft.pop_back();
        pathsEdgeRight.pop_back();
     
        break;
    }

    case ParkStep::out: 
    {
        if (Parkstep == ParkType::ParkRight)
        {
            if (!track.PointsEdgeRight_BA.empty() && track.PointsEdgeRight_BA.back().x < 30
                && isStraightLine(mapping.PointsEdgeRight) == true)
            {
                counterRec = 0;
                counterSession = 0;
                step = ParkStep::none;   // 退出状态.
                startTurning = false;    // 恢复状态
                garageFirst = true;      // 进入一号车库
                lineY = 0;               // 直线高度
                lineX = 0;
                ptA = Point(0, 0);       // 清空线段的两个端点
                ptB = Point(0, 0);
                pt_angle = 0;
                pathsEdgeRight.clear();
                pathsEdgeLeft.clear();
                break;
            }
            else
            {
                mapping.direction = LEFT;                
            }
        }
    }

    }
    return true;
    
}

void Parking::drawImage(Tracking track, Mat& image)
{
    // 赛道边缘
    for (size_t i = 0; i < track.PointsEdgeLeft_BA.size(); i++)
    {
        circle(image, Point(track.PointsEdgeLeft_BA[i].y, track.PointsEdgeLeft_BA[i].x), 1,
            Scalar(0, 255, 0), -1); // 绿色点
    }
    for (size_t i = 0; i < track.PointsEdgeRight_BA.size(); i++)
    {
        circle(image, Point(track.PointsEdgeRight_BA[i].y, track.PointsEdgeRight_BA[i].x), 1,
            Scalar(0, 255, 255), -1); // 黄色点
    }

    if (step != ParkStep::none)
        putText(image, "[1] BATTERY - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

}