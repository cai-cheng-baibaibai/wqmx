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
    //direction = 0:Ĭ�����ұ�

	int length = 1;

    const int min_distance_sq = 14; 
    const int right_boundary = COLSIMAGE - kernel - 1;

    if (tracking.PointsEdgeLeft_BA.size() > length)
    {
        const auto& leftBA = tracking.PointsEdgeLeft_BA;
        PointsEdgeLeft.reserve(tracking.PointsEdgeLeft_BA.size() / length);

        // ��ʼ�㴦��
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

        // ��ʼ�㴦��
        PointsEdgeRight.push_back(Cornerdetection(rightBA[0]));

        // ��ѭ���Ż�����Ϊ����ʽ����
        for (size_t i = 1; i < rightBA.size() - 2; i += 1) { // ���������
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

	////�Ⱦ����
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
			Scalar(255, 255, 255), -1); // �׵�
	}
	for (int i = 0; i < PointsEdgeRight.size(); i++)
	{
		circle(trackImage, Point(PointsEdgeRight[i].x, PointsEdgeRight[i].y), 1,
			Scalar(255, 255, 255), -1); // �׵�
	}
    //for (int i = 0; i < testpoints.size(); i++)
    //{
    //    circle(trackImage, Point(testpoints[i].x, testpoints[i].y), 4,
    //        Scalar(255, 0, 255), -1); // �׵�
    //}
    if (Left_jiaodian.jiaodian.x != 0)
    {
        circle(trackImage, Point(Left_jiaodian.jiaodian.x, Left_jiaodian.jiaodian.y), 4,
            Scalar(0, 0, 255), -1); // �׵�
    }
    if (Right_jiaodian.jiaodian.x != 0)
    {
        circle(trackImage, Point(Right_jiaodian.jiaodian.x, Right_jiaodian.jiaodian.y), 4,
            Scalar(0, 0, 255), -1); // �׵�
    }
   
}



Point PerspectiveMapping::Cornerdetection(const Point point)
{
    cv::Mat homogeneousInput = (cv::Mat_<double>(3, 1) << (double)point.y, (double)point.x, 1.0);
    cv::Mat homogeneousOutput = m_H * homogeneousInput;  // ����˷�

    // 2. ��һ����������ܵĳ������
    if (std::abs(homogeneousOutput.at<double>(2)) < 1e-10) {
        return cv::Point(-1, -1);
    }

    // 3. ֱ�ӷ��� Mat ���ݣ�������ʱ������
    double x = homogeneousOutput.at<double>(0) / homogeneousOutput.at<double>(2);
    double y = homogeneousOutput.at<double>(1) / homogeneousOutput.at<double>(2);

    return cv::Point(std::round(x), std::round(y));  // ע�⽻�� x/y
}

cv::Point PerspectiveMapping::InverseCornerDetection(const cv::Point& transformedPoint) {
   
    cv::Mat input = (cv::Mat_<double>(3, 1) <<
        transformedPoint.x,  // ��ȷ��x����
        transformedPoint.y,  // ��ȷ��y����
        1.0);

    cv::Mat output = m_H_inv * input;

    if (std::abs(output.at<double>(2)) < 1e-10) {
        std::cerr << "Warning: Division by near-zero in inverse transform!" << std::endl;
        return cv::Point(-1, -1);
    }

    return cv::Point( 
        std::round(output.at<double>(1) / output.at<double>(2)) ,  // y����
        std::round(output.at<double>(0) / output.at<double>(2))    // x����
    );
}

//vector<Point> PerspectiveMapping::Cornerdetection(const vector<Point> inputPoints)
//{
//    vector<Point> outputPoints;
//
//    // ���������ÿ����
//    for (const auto& inputPoint : inputPoints) {
//        // ����ά����ת��Ϊ������꣨���ӵ���άΪ1��
//        Mat homogeneousInput = (Mat_<double>(3, 1) << inputPoint.y, inputPoint.x, 1);
//
//        // ִ�о���˷���ģ��͸�ӱ任
//        Mat homogeneousOutput = m_H * homogeneousInput; // m_H �� 3x3 ͸�ӱ任����
//
//        // �����������ת���ض�ά���꣨���Ե���ά��ֵ����һ����
//        double w = homogeneousOutput.at<double>(2); // �������ĵ���ά
//        if (w != 0) {
//            double x = homogeneousOutput.at<double>(0) / w;
//            double y = homogeneousOutput.at<double>(1) / w;
//            outputPoints.push_back(Point2d(x, y)); // �洢͸�ӱ任��ĵ�
//        }
//        else {
//            // ���wΪ�㣬����ѡ��������ʹ����������ʽ
//            // ������һ���򵥵��ݴ���
//            outputPoints.push_back(Point2d(0, 0)); // ���Ը����������
//        }
//    }
//
//    return outputPoints;
//}

vector<Point> PerspectiveMapping::Cornerdetection(const vector<Point> inputPoints) {
    CV_Assert(m_H.type() == CV_64F && m_H.rows == 3 && m_H.cols == 3);

    // ������������
    Mat inputHomogeneous(inputPoints.size(), 3, CV_64F);
    for (size_t i = 0; i < inputPoints.size(); ++i) {
        inputHomogeneous.at<double>(i, 0) = inputPoints[i].y;
        inputHomogeneous.at<double>(i, 1) = inputPoints[i].x;
        inputHomogeneous.at<double>(i, 2) = 1;
    }

    Mat outputHomogeneous = inputHomogeneous * m_H.t();
    vector<Point> outputPoints;
    outputPoints.reserve(inputPoints.size());

    // ��һ�������
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
    const int WINDOW_SIZE = 5;        // �������ڴ�С
    const double MIN_ANGLE_DEG = 180; // ��С�Ƕ���ֵ���ȣ�

   
    Right_jiaodian = { Point(0, 0), 180.0 };
    Left_jiaodian = { Point(0, 0), 180.0 };
   // testpoints.clear();

    // ������߽�ǵ�
    findSingleSideJiaodian(PointsEdgeLeft, Left_jiaodian, true, WINDOW_SIZE, MIN_ANGLE_DEG);

    // ����Ҳ�߽�ǵ�
    findSingleSideJiaodian(PointsEdgeRight, Right_jiaodian, false, WINDOW_SIZE, MIN_ANGLE_DEG);
    
}

// ��������������߽�ǵ���
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

    // ֻ���ҵ����������ĵ�Ÿ������
    if(isLeft)
        output = found ? JiaoDian{ bestPoint, bestAngle } : JiaoDian{ Point(0,0), 180.0 };
    else
    {
        output = found ? JiaoDian{ bestPoint,360 -  bestAngle } : JiaoDian{ Point(0,0),180.0 };
    }
}

