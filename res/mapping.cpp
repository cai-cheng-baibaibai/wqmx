#include "../include/mapping.h"
using namespace cv;
using namespace std;

PerspectiveMapping::PerspectiveMapping()
{
    Point x1 (0, 0);
    Point x2 (ROWSIMAGE - 1, 0);
   
    Point x3 = Cornerdetection(x1);
    Point x4 = Cornerdetection(x2);

  /*  Point x5(ROWSIMAGE - 1, COLSIMAGE - 1);
    Point x6 = Cornerdetection(x5);*/

    m_H_inv = m_H.inv();

    int length = abs(x3.y - x4.y);
    Pixel_Pitch = 0.5421;
}

Mat PerspectiveMapping::get_m_H()
{
    return m_H;
}

double PerspectiveMapping::PixelPitch()
{
    return Pixel_Pitch;
}

void PerspectiveMapping::Edgehandling(Tracking& tracking)
{
	PointsEdgeLeft.clear();
	PointsEdgeRight.clear();
	//tracking.PointsEdge_extract();
    //direction = 0:默认是右边

	int length = 1;

    const int min_distance_sq = 14; 
    const int right_boundary = COLSIMAGE - kernel - 1;

    if (tracking.PointsEdgeLeft_BA.size() > length)
    {
        const auto& leftBA = tracking.PointsEdgeLeft_BA;
        PointsEdgeLeft.reserve(tracking.PointsEdgeLeft_BA.size() / length);

        // 初始点处理
        PointsEdgeLeft.push_back(Cornerdetection(leftBA[0]));

        for (int i = 1; i < tracking.PointsEdgeLeft_BA.size() - length; i ++)
        {
            const Point& current = Cornerdetection(leftBA[i]);
            const Point& prev = PointsEdgeLeft.back();

            const int dx = current.x - prev.x;
            const int dy = current.y - prev.y;
            const int dist_sq = dx * dx + dy * dy;

            if (leftBA[i].y > kernel && dist_sq >= min_distance_sq) {
                PointsEdgeLeft.push_back(current);
            }
        }

    }
    if (tracking.PointsEdgeRight_BA.size() > length)
    {
        const auto& rightBA = tracking.PointsEdgeRight_BA;
        PointsEdgeRight.reserve(rightBA.size() / 2);

        // 初始点处理
        PointsEdgeRight.push_back(Cornerdetection(rightBA[0]));

        // 主循环优化（改为步进式处理）
        for (size_t i = 1; i < rightBA.size() - 2; i += 1) { // 保持逐点检查
            const Point& current = Cornerdetection(rightBA[i]);
            const Point& prev = PointsEdgeRight.back();

            const int dx = current.x - prev.x;
            const int dy = current.y - prev.y;
            const int dist_sq = dx * dx + dy * dy;

            if (rightBA[i].y < right_boundary && dist_sq >= min_distance_sq) {
                PointsEdgeRight.push_back(current);
            }
        }

    }
	

    findPointsjiaodian();
	
    /*PointsEdgeLeft = tracking.smoothTrajectory(PointsEdgeLeft, 25, 9.0);
    PointsEdgeRight = tracking.smoothTrajectory(PointsEdgeRight, 25, 9.0);*/

	////等距采样
	//if (tempPointsEdgeLeft.size() != 0)
	//{
	//	PointsEdgeLeft.push_back(tempPointsEdgeLeft[0]);
	//	for (int i = 0; i < tempPointsEdgeLeft.size(); i++)
	//	{
	//		Point pointtemp = PointsEdgeLeft[PointsEdgeLeft.size() - 1];
	//		double point_Distance = sqrt((pointtemp.x - tempPointsEdgeLeft[i].x) * (pointtemp.x - tempPointsEdgeLeft[i].x) +
	//			(pointtemp.y - tempPointsEdgeLeft[i].y) * (pointtemp.y - tempPointsEdgeLeft[i].y));
	//		double Pixel_Pitch = mapping.PixelPitch();
	//		if (point_Distance * Pixel_Pitch > 2)
	//			PointsEdgeLeft.push_back(tempPointsEdgeLeft[i]);
	//	}
	//}
	//
	//if (tempPointsEdgeRight.size() != 0)
	//{
	//	PointsEdgeRight.push_back(tempPointsEdgeRight[0]);
	//	for (int i = 0; i < tempPointsEdgeRight.size(); i++)
	//	{
	//		Point pointtemp = PointsEdgeRight[PointsEdgeRight.size() - 1];
	//		double point_Distance = sqrt((pointtemp.x - tempPointsEdgeRight[i].x) * (pointtemp.x - tempPointsEdgeRight[i].x) +
	//			(pointtemp.y - tempPointsEdgeRight[i].y) * (pointtemp.y - tempPointsEdgeRight[i].y));
	//		double Pixel_Pitch = mapping.PixelPitch();
	//		if (point_Distance * Pixel_Pitch > 2)
	//			PointsEdgeRight.push_back(tempPointsEdgeRight[i]);
	//	}
	//}
}


void PerspectiveMapping::drawImage(Mat& trackImage)
{
	for (int i = 0; i < PointsEdgeLeft.size(); i++)
	{
		circle(trackImage, Point(PointsEdgeLeft[i].x, PointsEdgeLeft[i].y), 1,
			Scalar(255, 255, 255), -1); // 白点
	}
	for (int i = 0; i < PointsEdgeRight.size(); i++)
	{
		circle(trackImage, Point(PointsEdgeRight[i].x, PointsEdgeRight[i].y), 1,
			Scalar(255, 255, 255), -1); // 白点
	}
    //for (int i = 0; i < testpoints.size(); i++)
    //{
    //    circle(trackImage, Point(testpoints[i].x, testpoints[i].y), 4,
    //        Scalar(255, 0, 255), -1); // 白点
    //}
    if (Left_jiaodian.jiaodian.x != 0)
    {
        circle(trackImage, Point(Left_jiaodian.jiaodian.x, Left_jiaodian.jiaodian.y), 4,
            Scalar(0, 0, 255), -1); // 白点
    }
    if (Right_jiaodian.jiaodian.x != 0)
    {
        circle(trackImage, Point(Right_jiaodian.jiaodian.x, Right_jiaodian.jiaodian.y), 4,
            Scalar(0, 0, 255), -1); // 白点
    }
   
}



Point PerspectiveMapping::Cornerdetection(const Point point)
{
    cv::Mat homogeneousInput = (cv::Mat_<double>(3, 1) << (double)point.y, (double)point.x, 1.0);
    cv::Mat homogeneousOutput = m_H * homogeneousInput;  // 矩阵乘法

    // 2. 归一化并处理可能的除零错误
    if (std::abs(homogeneousOutput.at<double>(2)) < 1e-10) {
        return cv::Point(-1, -1);
    }

    // 3. 直接访问 Mat 数据（避免临时变量）
    double x = homogeneousOutput.at<double>(0) / homogeneousOutput.at<double>(2);
    double y = homogeneousOutput.at<double>(1) / homogeneousOutput.at<double>(2);

    return cv::Point(std::round(x), std::round(y));  // 注意交换 x/y
}

cv::Point PerspectiveMapping::InverseCornerDetection(const cv::Point& transformedPoint) {
   
    cv::Mat input = (cv::Mat_<double>(3, 1) <<
        transformedPoint.x,  // 正确的x坐标
        transformedPoint.y,  // 正确的y坐标
        1.0);

    cv::Mat output = m_H_inv * input;

    if (std::abs(output.at<double>(2)) < 1e-10) {
        std::cerr << "Warning: Division by near-zero in inverse transform!" << std::endl;
        return cv::Point(-1, -1);
    }

    return cv::Point( 
        std::round(output.at<double>(1) / output.at<double>(2)) ,  // y坐标
        std::round(output.at<double>(0) / output.at<double>(2))    // x坐标
    );
}

//vector<Point> PerspectiveMapping::Cornerdetection(const vector<Point> inputPoints)
//{
//    vector<Point> outputPoints;
//
//    // 遍历输入的每个点
//    for (const auto& inputPoint : inputPoints) {
//        // 将二维坐标转换为齐次坐标（增加第三维为1）
//        Mat homogeneousInput = (Mat_<double>(3, 1) << inputPoint.y, inputPoint.x, 1);
//
//        // 执行矩阵乘法，模拟透视变换
//        Mat homogeneousOutput = m_H * homogeneousInput; // m_H 是 3x3 透视变换矩阵
//
//        // 将齐次坐标结果转换回二维坐标（除以第三维的值来归一化）
//        double w = homogeneousOutput.at<double>(2); // 齐次坐标的第三维
//        if (w != 0) {
//            double x = homogeneousOutput.at<double>(0) / w;
//            double y = homogeneousOutput.at<double>(1) / w;
//            outputPoints.push_back(Point2d(x, y)); // 存储透视变换后的点
//        }
//        else {
//            // 如果w为零，可以选择跳过或使用其他处理方式
//            // 这里是一个简单的容错处理
//            outputPoints.push_back(Point2d(0, 0)); // 可以根据需求调整
//        }
//    }
//
//    return outputPoints;
//}

vector<Point> PerspectiveMapping::Cornerdetection(const vector<Point> inputPoints) {
    CV_Assert(m_H.type() == CV_64F && m_H.rows == 3 && m_H.cols == 3);

    // 批量矩阵运算
    Mat inputHomogeneous(inputPoints.size(), 3, CV_64F);
    for (size_t i = 0; i < inputPoints.size(); ++i) {
        inputHomogeneous.at<double>(i, 0) = inputPoints[i].y;
        inputHomogeneous.at<double>(i, 1) = inputPoints[i].x;
        inputHomogeneous.at<double>(i, 2) = 1;
    }

    Mat outputHomogeneous = inputHomogeneous * m_H.t();
    vector<Point> outputPoints;
    outputPoints.reserve(inputPoints.size());

    // 归一化与过滤
    for (int i = 0; i < outputHomogeneous.rows; ++i) {
        double w = outputHomogeneous.at<double>(i, 2);
        if (std::abs(w) > 1e-6) {
            outputPoints.emplace_back(
                outputHomogeneous.at<double>(i, 0) / w,
                outputHomogeneous.at<double>(i, 1) / w
            );
        }
    }


    return outputPoints;
}

void PerspectiveMapping::findPointsjiaodian()
{
    const int WINDOW_SIZE = 5;        // 滑动窗口大小
    const double MIN_ANGLE_DEG = 180; // 最小角度阈值（度）

   
    Right_jiaodian = { Point(0, 0), 180.0 };
    Left_jiaodian = { Point(0, 0), 180.0 };
   // testpoints.clear();

    // 检测左侧边界角点
    findSingleSideJiaodian(PointsEdgeLeft, Left_jiaodian, true, WINDOW_SIZE, MIN_ANGLE_DEG);

    // 检测右侧边界角点
    findSingleSideJiaodian(PointsEdgeRight, Right_jiaodian, false, WINDOW_SIZE, MIN_ANGLE_DEG);
    
}

// 辅助函数：单侧边界角点检测
void PerspectiveMapping::findSingleSideJiaodian(const vector<Point>& edgePoints, JiaoDian& output, bool isLeft, int windowSize, double minAngleDeg) {
    if (edgePoints.size() < 2 * windowSize + 1) return;

  
    double bestAngle = isLeft ? 180.0 : 0.0;
    Point bestPoint(0, 0);
    bool found = false;

    for (int i = windowSize; i < edgePoints.size() - windowSize; ++i) {
        Point prevVec = edgePoints[i] - edgePoints[i - windowSize];
        Point nextVec = edgePoints[i] - edgePoints[i + windowSize];

        double angle = calculateAngle(prevVec, nextVec) * 180.0 / CV_PI;

        bool isBetter = isLeft ? (angle < bestAngle) : (angle > bestAngle);

        if (isBetter) {
            bestAngle = angle;
            bestPoint = edgePoints[i];
            found = true;
        }
    }

    // 只有找到符合条件的点才更新输出
    if(isLeft)
        output = found ? JiaoDian{ bestPoint, bestAngle } : JiaoDian{ Point(0,0), 180.0 };
    else
    {
        output = found ? JiaoDian{ bestPoint,360 -  bestAngle } : JiaoDian{ Point(0,0),180.0 };
    }
}

