#include "../../include/parking.h"


bool Parking::process(Tracking& track, Mat& image, PerspectiveMapping& mapping, vector<PredictResult> predict)
{
    counterSession++;
    //if (step != ParkStep::none && counterSession > 80) // 超时退出
    //{
    //    counterRec = 0;
    //    counterSession = 0;
    //    step = ParkStep::none;   // 退出状态.
    //    startTurning = false;    // 恢复状态
    //    garageFirst = true;      // 进入一号车库
    // find_First = false;
    //    lineY = 0;               // 直线高度
    //    ptA = Point(0, 0);       // 清空线段的两个端点
    //    ptB = Point(0, 0);
    //    std::cout << "退出停车场" << std::endl;
    //}
    switch (step)
    {
    case ParkStep::none: // AI未识别
    {
        for (size_t i = 0; i < predict.size(); i++)
        {
            if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.4)
            {
                counterRec++;
                break;
            }
        }
        if (counterRec) // 检测到一帧后开始连续监测AI标志是否满足条件
        {
            if (counterRec >= 3 && counterSession < 8)
            {
                counterRec = 0;
                counterSession = 0;
                step = ParkStep::enable; // 检测到停车场标志
                return true;
            }
            else if (counterSession >= 8)
            {
                counterRec = 0;
                counterSession = 0;
            }
        }
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
        /*Mat edges, dstImage;
        GaussianBlur(image, dstImage, Size(5, 5), 0, 0);
        Canny(dstImage, edges, 50, 150);*/
        Mat edges = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC1);
        if (Parkstep == ParkType::ParkRight)
        {

        }
        else
        {
            for (int i = 0; i < track.PointsEdgeLeft_BA.size(); i++)
            {
                circle(edges, Point(track.PointsEdgeLeft_BA[i].y, track.PointsEdgeLeft_BA[i].x), 1,
                    Scalar(255), -1); // 绿色点
            }
        }

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
            if (Parkstep == ParkType::ParkRight)
            {
                if (abs(angle) < 30 && angle < 0 && midY < 200 && midX > COLSIMAGE / 2)
                { // 接近水平
                    horizontalLines.push_back(line);
                    break;
                    //cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                }
            }
            else
            {
                if (abs(angle) < 30 && angle > 0 && midX < COLSIMAGE / 2 && midY < 200)
                { // 接近水平
                    horizontalLines.push_back(line);
                    break;
                    //cv::line(imgRes, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);
                }
            }
         
        }

        Mat imgRes1 = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        if (!horizontalLines.empty())
        {
            /*if (front_line[0] != 0)
            {
                Point pt1(front_line[0], front_line[1]);
                Point pt2(front_line[2], front_line[3]);
                int front_line_midX = (front_line[0] + front_line[2]) / 2;
                int front_line_midY = (front_line[1] + front_line[3]) / 2;
                double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
            }*/
            front_line = horizontalLines[0];
            cv::line(imgRes, Point(front_line[0], front_line[1]), Point(front_line[2], front_line[3]), Scalar(0, 0, 255), 2);
        }

        /*imshow("Detected Lines", imgRes);
        moveWindow("Detected Lines", 100, 100);
        waitKey(1);*/

        if (front_line[0] != 0)
        {
            int midY = (front_line[1] + front_line[3]) / 2;
            if (1)
            {
                //garageFirst = true;         // 进入一号车库(远)
                find_First = false;
                lineY = (front_line[1] + front_line[3]) / 2;;  // 获取距离最远的线控制车入库
                lineX = (front_line[0] + front_line[2]) / 2;;  // 获取距离最远的线控制车入库
                step = ParkStep::turning; // 开始入库
                counterSession = 0;
            }
            //else
            //{
            //    garageFirst = true;         // 进入一号车库(远)
            // find_First = false;
            //    lineY = (front_line[1] + front_line[3]) / 2;  // 获取距离最远的线控制车入库
            //    lineX = (front_line[0] + front_line[2]) / 2;  // 获取距离最远的线控制车入库
            //    step = ParkStep::turning; // 开始入库
            //    counterSession = 0;
            //}

            break;
        }


      /*  imshow("imgRes1", imgRes1);
        moveWindow("imgRes1", 100, 700);
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

        std::sort(lines.begin(), lines.end(), [](const Vec4i& a, const Vec4i& b) {
            double midY1 = (a[1] + a[3]) / 2.0;
            double midY2 = (b[1] + b[3]) / 2.0;
            return midY1 < midY2;
            });

        Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        for (const Vec4i& line : lines) {
            Point pt1(line[0], line[1]);
            Point pt2(line[2], line[3]);

            int midX = (line[0] + line[2]) / 2;
            int midY = (line[1] + line[3]) / 2;
            double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;

            if (!startTurning)
            {
                // 接近水平并且在右侧
                if (Parkstep == ParkType::ParkRight)
                {
                    if (abs(angle) < 40 && angle < 0 && midX > COLSIMAGE / 2)
                    {
                        if (midY > lineY && (midY - lineY) <= 10) // 限制线段增加值
                        {
                            lineX = midX;
                            lineY = midY; // 更新直线高度
                            ptA = pt1;    // 更新端点
                            ptB = pt2;
                            pt_angle = angle;
                            break;
                        }
                    }
                }
                else if (abs(angle) < 40 && angle > 0 && midX < COLSIMAGE / 2)
                {
                    if ( midY > lineY && (midY - lineY) <= 5) // 限制线段增加值
                    {
                        lineX = midX;
                        lineY = midY; // 更新直线高度
                        ptA = pt1;    // 更新端点
                        ptB = pt2;
                        pt_angle = angle;
                        break;
                    }
                }
            }
            else
            {
                if (Parkstep == ParkType::ParkRight)
                {
                    if (abs(midY - lineY) <= 15 && abs(pt_angle - angle) < 15 && abs(midX - lineX) < 15)
                    {
                        lineX = midX;
                        lineY = midY; // 更新直线高度
                        ptA = pt1;    // 更新端点
                        ptB = pt2;
                        pt_angle = angle;
                    }
                }
                else
                {
                    if (abs(midY - lineY) <= 15 && abs(pt_angle - angle) < 15 && abs(midX - lineX) < 15)
                    {
                        lineX = midX;
                        lineY = midY; // 更新直线高度
                        ptA = pt1;    // 更新端点
                        ptB = pt2;
                        pt_angle = angle;
                    }
                    
                }
            }
          
        }

        cv::line(imgRes, ptA, ptB, Scalar(0, 0, 255), 2);
        
        if (lineY > 130 && !garageFirst) // 控制转弯时机
        {
            if (!find_First)
            {
                cv::Mat line = track.Target_Image.col(lineX).clone();
                cv::Mat binary_line;  // 预定义输出矩阵
                double average = cv::threshold(
                    line,             // 输入图像
                    binary_line,      // 输出图像（必须有效）
                    0,                // 阈值（OTSU自动计算，此处无效）
                    255,              // 最大值
                    cv::THRESH_BINARY | cv::THRESH_OTSU  // 组合阈值类型
                );
                //cout << line << endl;

                int cnt = 0;
                for (int i = 0; i < 70; i++)
                {
                    if (track.Target_Image.at<uchar>(lineY - i, lineX) < average)
                    {
                        cnt++;
                        //cout << track.Target_Image.at<uchar>(lineY - i, lineX) << endl;;
                    }
                }
                if (cnt > 30)
                {
                    garageFirst = false;
                }
                else
                {
                    garageFirst = true;
                }
                find_First = true;
            }
           

            if (!garageFirst)
            {
                for (const Vec4i& line : lines)
                {
                    Point pt1(line[0], line[1]);
                    Point pt2(line[2], line[3]);

                    int midX = (line[0] + line[2]) / 2;
                    int midY = (line[1] + line[3]) / 2;
                    double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
                    //cv::line(imgRes, pt1, pt2, Scalar(0, 0, 255), 2);
                    // && abs(pt_angle - angle) < 20 && midY < lineY && abs(midY - lineY) <= 70 && abs(midY - lineY) >= 30  && midX < COLSIMAGE / 2
                    if (abs(pt_angle - angle) < 20 && midX < COLSIMAGE / 2 && abs(midY - lineY) <= 90 && abs(midY - lineY) >= 50)
                    {
                        cv::line(imgRes, pt1, pt2, Scalar(0, 0, 255), 2);
                        lineX = midX;
                        lineY = midY; // 更新直线高度
                        ptA = pt1;    // 更新端点
                        ptB = pt2;
                        pt_angle = angle;
                        garageFirst = true;
                        break;
                    }
                }
                imshow("Detected Lines", imgRes);
                waitKey(1);
                int i = 0;

            }
        }

        imshow("Detected Lines", imgRes);
        waitKey(1);


        if (lineY > 130 || startTurning) // 控制转弯时机
        {
            if (!startTurning)
            {
                counterSession = 0;
                startTurning = true; // 已经开始转弯      
            }
            //std::cout << "控制转弯" << std::endl;

            if (Parkstep == ParkType::ParkRight)
            {

            }
            else
            {
                // 计算直线的斜率
                double slope = static_cast<double>(ptB.y - ptA.y) / (ptB.x - ptA.x + 1e-5); // 避免除零
                int y3 = slope * (0 - ptA.x) + ptA.y;        // 延长起点的Y坐标
                int y4 = slope * (COLSIMAGE / 2 - ptA.x) + ptA.y;// 延长终点的Y坐标

                Point start(0, y3);           // 延长起点
                Point end(COLSIMAGE / 2, y4);     // 延长终点
                    
               /* Point start(ptA.x, ptA.y);
                Point end(ptB.x, ptB.y);*/

                track.PointsEdgeLeft_BA.clear(); // 清空原始点集

                for(int x = end.x; x >= start.x; x--) {
                    int y = static_cast<int>(start.y + slope * (x - start.x)); // 根据斜率计算 y 值
                    Point pt;
                    pt.x = y; // 将 cv::Point 的 x 赋值给 POINT 的 y
                    pt.y = x; // 将 cv::Point 的 y 赋值给 POINT 的 x
                    if (pt.x < 0 || pt.y < 0)
                        break;
                    track.PointsEdgeLeft_BA.push_back(pt); // 将 POINT 存入点集
                }
                mapping.PointsEdgeLeft.clear();
                mapping.PointsEdgeLeft = mapping.Cornerdetection(track.PointsEdgeLeft_BA);
                mapping.direction = LEFT;
            }

            /*imshow("Detected Lines", imgRes);
            waitKey(1);*/


            pathsEdgeLeft.push_back(mapping.PointsEdgeLeft); // 记录进厂轨迹
            pathsEdgeRight.push_back(mapping.PointsEdgeRight);
        }
        if (counterSession > truningTime && startTurning) // 开始停车状态
        {
            //std::cout << "开始停车" << std::endl;
            step = ParkStep::stop; // 开始停车
        }
 
        ////已经开始转弯,判断停止
        //Mat imgRes1 = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
        //if (startTurning)
        //{
        //    HoughLinesP(edges, lines, 1, CV_PI / 180, 40, 40, 20);
        //    for (const Vec4i& line : lines)
        //    {
        //        Point pt1(line[0], line[1]);
        //        Point pt2(line[2], line[3]);

        //        int midX = (line[0] + line[2]) / 2;
        //        int midY = (line[1] + line[3]) / 2;
        //        double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;

        //        if (angle > 0 && abs(angle) < 20 && midY < void_row && abs(midY - void_row) < 20 && midX < COLSIMAGE / 2)
        //        {
        //            std::cout << "stopping" << std::endl;
        //            step = ParkStep::stop; // 开始停车
        //            counterSession = 0;
        //        }
        //        
        //    }
        //   /* for (int i = 0; i < lines.size(); i++)
        //    {
        //        cv::line(imgRes1, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 2);
        //    }*/
        //}

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
            find_First = false;
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
        //if (isStraightLine(mapping.PointsEdgeLeft) == true)
        //{
        //    if (!track.PointsEdgeLeft_BA.empty() && track.PointsEdgeLeft_BA.back().x < 30)
        //    {
        //        counterRec = 0;
        //        counterSession = 0;
        //        step = ParkStep::out;   // 退出状态.
        //        startTurning = false;    // 恢复状态
        //        garageFirst = true;      // 进入一号车库
        // find_First = false;
        //        lineY = 0;               // 直线高度
        //        lineX = 0;
        //        ptA = Point(0, 0);       // 清空线段的两个端点
        //        ptB = Point(0, 0);
        //        pt_angle = 0;
        //        pathsEdgeRight.clear();
        //        pathsEdgeLeft.clear();
        //        break;
        //    }
        //}

        //if (counterSession > 40 && (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1))
        //{
        //    counterRec = 0;
        //    counterSession = 0;
        //    step = ParkStep::out;   // 退出状态.
        //    startTurning = false;    // 恢复状态
        //    garageFirst = true;      // 进入一号车库
        // find_First = false;
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
            mapping.direction = LEFT;
            if (counterSession > 60)
            {
                counterSession = 0;
                step = ParkStep::none;
            }
        }
        else
        {
            mapping.direction = RIGHT;
            if (counterSession > 60)
            {
                counterSession = 0;
                step = ParkStep::none;
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