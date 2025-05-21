#include"../../include/tracking.h"
using namespace cv;
using namespace std;


void Tracking::trackRecognition(Mat& image, int row,Mat m_h,double PixelPitch)
{
    
    Target_Image = image;
    rectangle(Target_Image, Point(void_col_min, void_row), Point(void_col_max, void_row), Scalar(255, 255, 255), 1);
    rectangle(Target_Image, Point(void_col_min, void_row), Point(void_col_min, ROWSIMAGE - 1), Scalar(255, 255, 255), 1);
    rectangle(Target_Image, Point(void_col_max, void_row), Point(void_col_max, ROWSIMAGE - 1), Scalar(255, 255, 255), 1);
    rowStart = row;
    m_H = m_h;
    pix = PixelPitch;
    trackRecognition();

}

void Tracking::trackRecognition()
{
    PointsEdgeLeft_BA.clear();              // ��ʼ����Ե���
    PointsEdgeRight_BA.clear();             // ��ʼ����Ե���
    yuan_PointsEdgeLeft_BA.clear();
    yuan_PointsEdgeRight_BA.clear();
    midline.clear();

    startingPoints.left = Point(0, 0);
    startingPoints.right = Point(0, 0);

    Searching_for_startingPoints();
    if (startingPoints.left.x != 0)
    {
        //��
        int x0 = startingPoints.left.x, y0 = startingPoints.left.y;
        int x1, y1, x2, y2;
        for (int i = 0; i < 16; i++)
        {
            x1 = x0 + seeds_l[i][0];     y1 = y0 + seeds_l[i][1];
            x2 = x0 + seeds_l[i + 1][0]; y2 = y0 + seeds_l[i + 1][1];
            Point Pointemp1(x1, y1);
            Point Pointemp2(x2, y2);
            /*if (y2 == 41);
            {
                cout << "ok" << endl;
            }*/
            if (x2 < rowCutUp || x2 > ROWSIMAGE - 5 || PointsEdgeLeft_BA.size() > 800 || y2 <= kernel || y2 >= COLSIMAGE - kernel - 1
                || Pointemp2 == startingPoints.left)
            {
                break;
            }
            if (isEdgePoint(Pointemp1) == 0 &&
                isEdgePoint(Pointemp2) == 1)
            {
                PointsEdgeLeft_BA.push_back(Pointemp2);
                i = (i + 4) % 8;
                x0 = x2; y0 = y2;
            }
        }
    }
    if (startingPoints.right.x != 0 )
    {
        //��
        int x0 = startingPoints.right.x, y0 = startingPoints.right.y;
        int x1, y1, x2, y2;
        for (int i = 0; i < 16; i++)
        {
            x1 = x0 + seeds_r[i][0];     y1 = y0 + seeds_r[i][1];
            x2 = x0 + seeds_r[i + 1][0]; y2 = y0 + seeds_r[i + 1][1];
            Point Pointemp1(x1, y1);
            Point Pointemp2(x2, y2);
            /*if (y2 == 41);
            {
                cout << "ok" << endl;
            }*/
            if (x2 < rowCutUp || x2 > ROWSIMAGE - 5 || PointsEdgeRight_BA.size() > 800 ||  y2 >= COLSIMAGE - kernel - 1 || y2 <= kernel
                || Pointemp2 == startingPoints.right)
            {
                break;
            }
            if (Pointemp2 == startingPoints.right )
            {
                break;
            }

            if (isEdgePoint(Pointemp1) == 0 &&
                isEdgePoint(Pointemp2) == 1)
            {
                PointsEdgeRight_BA.push_back(Pointemp2);
                i = (i + 4) % 8;
                x0 = x2; y0 = y2;
            }
        }
    }
        //��ȡ������Ե
        //PointsEdge_extract();
        // ����߽�
        //Edgecleaning();
        //Extractedge();
        /*if (PointsEdgeLeft_BA.size() != 0 && PointsEdgeRight_BA.size() != 0)
        {
        }*/
   /* PointsEdgeLeft_BA = smoothTrajectory(PointsEdgeLeft_BA, 25, 9.0);
    PointsEdgeRight_BA = smoothTrajectory(PointsEdgeRight_BA, 25, 9.0);*/

    
    if(yuanxian)
        searching_yuanxian();
    
}

std::vector<cv::Point> Tracking::smoothTrajectory(const std::vector<cv::Point>& trajectory, int window_size, double sigma)
{
    std::vector<cv::Point> smoothed;
    if (trajectory.empty() || window_size < 1) return smoothed;

    const int n = trajectory.size();
    smoothed.reserve(n);

    // ��˹Ȩ�ؼ��㣨���������Ȩ�أ�
    std::vector<double> weights;
    weights.reserve(window_size);
    double sum_weights = 0.0;
    for (int i = 0; i < window_size; ++i) {
        double weight = exp(-(i * i) / (2 * sigma * sigma)); // ��˹��
        weights.push_back(weight);
        sum_weights += weight;
    }

    // �߽羵����䣨����߽�ʧ�棩
    auto get_mirrored_point = [&](int idx) {
        if (idx < 0) return trajectory[0];
        if (idx >= n) return trajectory[n - 1];
        return trajectory[idx];
        };

    // �������ڼ�Ȩƽ��
    for (int i = 0; i < n; ++i) {
        double sum_x = 0.0, sum_y = 0.0;
        double current_weight_sum = 0.0;

        for (int j = -window_size / 2; j <= window_size / 2; ++j) {
            const auto& pt = get_mirrored_point(i + j);
            double w = weights[abs(j)];
            sum_x += pt.x * w;
            sum_y += pt.y * w;
            current_weight_sum += w;
        }

        smoothed.emplace_back(
            static_cast<int>(std::round(sum_x / current_weight_sum)),
            static_cast<int>(std::round(sum_y / current_weight_sum))
        );
    }

    return smoothed;
}

void Tracking::searching_yuanxian()
{
    startingPoints.left = Point(0, 0);
    startingPoints.right = Point(0, 0);

    int rowstart;

    if (PointsEdgeLeft_BA.empty() && PointsEdgeRight_BA.empty())
    {
        rowstart = ROWSIMAGE - 10;
    }
    else if(PointsEdgeLeft_BA.empty())
    {
        rowstart = PointsEdgeRight_BA.back().x - 10;
    }
    else if (PointsEdgeRight_BA.empty())
    {
        rowstart = PointsEdgeLeft_BA.back().x - 10;
    }
    else
    {
        rowstart = min(PointsEdgeLeft_BA.back().x, PointsEdgeRight_BA.back().x) - 5;
        //rowstart = PointsEdgeRight_BA.back().x - 10;
    }
    if (rowstart < 20) return;

    
    vector<Point> left_start;
    vector<Point> right_start;
    while (startingPoints.left.x == 0 && startingPoints.right.x == 0)
    {
        startingPoints.left = Point(0, 0);
        startingPoints.right = Point(0, 0);
        left_start.clear();
        right_start.clear();

        for (int col = 240; col >= kernel; col--)
        {
            if (col > void_col_max || col < void_col_min || rowstart < void_row)
            {
                Point Pointtemp1(rowstart, col); Point Pointtemp2(rowstart, col - 1);
                if (isEdgePoint(Pointtemp2) == 1 && isEdgePoint(Pointtemp1) == 0
                    && Target_Image.at<uchar>(rowstart, col - kernel) < Target_Image.at<uchar>(rowstart, col + kernel) - 20
                    )
                {
                    if (isStartingPoints(Pointtemp2, true) == true)
                    {
                        //yuan_PointsEdgeLeft_BA.push_back(Pointtemp2);
                        left_start.push_back(Pointtemp2);
                    }

                }

            }
        }
        for (int col = 80; col <= COLSIMAGE - 1 - kernel; col++)
        {
            if (col > void_col_max || col < void_col_min || rowstart < void_row)
            {
                Point Pointtemp1(rowstart, col); Point Pointtemp2(rowstart, col + 1);

                if (isEdgePoint(Pointtemp2) == 1 && isEdgePoint(Pointtemp1) == 0
                    && Target_Image.at<uchar>(rowstart, col - kernel) > Target_Image.at<uchar>(rowstart, col + kernel) + 20
                    )
                {
                    if (isStartingPoints(Pointtemp2, false) == true)
                    {
                        //yuan_PointsEdgeRight_BA.push_back(Pointtemp2);
                        right_start.push_back(Pointtemp2);
                    }

                }
            }
        }

        if (!left_start.empty() && !right_start.empty())
        {
            for (int i = 0; i < left_start.size(); i++)
            {
                const Point current = Cornerdetection(left_start[i], m_H);

                for (int j = 0; j < right_start.size(); j++)
                {
                    const Point prev = Cornerdetection(right_start[j], m_H);
                    double dis = pix * sqrt((current.x - prev.x) * (current.x - prev.x) + (current.y - prev.y) * (current.y - prev.y));

                    if (dis < 55 && dis > 35)
                    {
                        if (1)
                        {
                            
                            startingPoints.left = left_start[i];
                            startingPoints.right = right_start[j];
                        }
                      //  startingPoints.left = left_start[i];
                        //startingPoints.right = right_start[j];
                    }
                }
            }
        }


        rowstart -= 5;
        if (rowstart < 20)return;
    }
  

    if (startingPoints.left.x != 0)
    {
        //��
        int x0 = startingPoints.left.x, y0 = startingPoints.left.y;
        int x1, y1, x2, y2;
        for (int i = 0; i < 16; i++)
        {
            x1 = x0 + seeds_l[i][0];     y1 = y0 + seeds_l[i][1];
            x2 = x0 + seeds_l[i + 1][0]; y2 = y0 + seeds_l[i + 1][1];
            Point Pointemp1(x1, y1);
            Point Pointemp2(x2, y2);
            /*if (y2 == 41);
            {
                cout << "ok" << endl;
            }*/
            if (x2 < rowCutUp || x2 > ROWSIMAGE - 5 || PointsEdgeLeft_BA.size() > 800 || y2 <= kernel || y2 >= COLSIMAGE - kernel - 1
                || Pointemp2 == startingPoints.left)
            {
                break;
            }
            if (isEdgePoint(Pointemp1) == 0 &&
                isEdgePoint(Pointemp2) == 1)
            {
                yuan_PointsEdgeLeft_BA.push_back(Pointemp2);
                i = (i + 4) % 8;
                x0 = x2; y0 = y2;
            }
        }
    }

    if (startingPoints.right.x != 0)
    {
        //��
        int x0 = startingPoints.right.x, y0 = startingPoints.right.y;
        int x1, y1, x2, y2;
        for (int i = 0; i < 16; i++)
        {
            x1 = x0 + seeds_r[i][0];     y1 = y0 + seeds_r[i][1];
            x2 = x0 + seeds_r[i + 1][0]; y2 = y0 + seeds_r[i + 1][1];
            Point Pointemp1(x1, y1);
            Point Pointemp2(x2, y2);
            /*if (y2 == 41);
            {
                cout << "ok" << endl;
            }*/
            if (Pointemp2 == startingPoints.right)
            {
                break;
            }

            if (x2 < rowCutUp || x2 > ROWSIMAGE - 5 || yuan_PointsEdgeRight_BA.size() > 800 || y2 >= COLSIMAGE - kernel - 1 || y2 <= kernel
                || Pointemp2 == startingPoints.right)
            {
                break;
            }
            if (isEdgePoint(Pointemp1) == 0 &&
                isEdgePoint(Pointemp2) == 1)
            {
                yuan_PointsEdgeRight_BA.push_back(Pointemp2);

               
                i = (i + 4) % 8;
                x0 = x2; y0 = y2;
            }
        }
    }

}


void Tracking::Searching_for_startingPoints()
{
    vector<Point> left_start;
    vector<Point> right_start;

    int rowstart = rowStart;

    for (int col = 0; col <= COLSIMAGE - 1 ; col++)
    {        
        Point Pointtemp1(rowstart, col); Point Pointtemp2(rowstart, col + 1);
        if (isEdgePoint(Pointtemp2) == 1 && isEdgePoint(Pointtemp1) == 0)
        {
            left_start.push_back(Pointtemp2);
        }
    }


    for (int col = COLSIMAGE - 1; col >= 0; col--)
    {
       
        Point Pointtemp1(rowstart, col); Point Pointtemp2(rowstart, col + 1);
        if (isEdgePoint(Pointtemp2) == 0 && isEdgePoint(Pointtemp1) == 1)
        {
            right_start.push_back(Pointtemp2);
        }
    }
  /*  long average = 0;
    for (int col = 0; col < COLSIMAGE ; col++)
    {
        if (col > void_col_max || col < void_col_min || rowstart < void_row)
        {
            average += Target_Image.at<uchar>(rowstart, col);
        }
    }
    average /= COLSIMAGE;*/


    cv::Mat line = Target_Image.row(rowstart).clone();
    cv::Mat binary_line;  // Ԥ�����������
    double average = cv::threshold(
        line,             // ����ͼ��
        binary_line,      // ���ͼ�񣨱�����Ч��
        0,                // ��ֵ��OTSU�Զ����㣬�˴���Ч��
        255,              // ���ֵ
        cv::THRESH_BINARY | cv::THRESH_OTSU  // �����ֵ����
    );
    //cout << line << endl;
    vector<Point> left_start_again;
    vector<Point> right_start_again;
    
    int detection_width = 40;
    int boundary_cnt_max = detection_width * 1.3;
    // ��߽�����⣨�Գ��߼���
    if (left_start.size() != 0)
    {
        for (int i = 0; i < left_start.size(); i++)
        {
            int boundary_cnt = 0;
            int x = left_start[i].x;
            int y = left_start[i].y;

            // �����ⰵ���߽磨����ƽ��ֵΪ��Ч��
            for (int j = 0; j < detection_width; j++)
            {
                if (y - j <= 0) break;
                if (y - j < void_col_max && y - j > void_col_min && x > void_row)
                {
                    continue;
                }
                else if (Target_Image.at<uchar>(x, y - j) < average)
                    boundary_cnt++;
            }

            // ���Ҽ�������߽磨����ƽ��ֵΪ��Ч��
            for (int j = 0; j < detection_width; j++)
            {
                if (y + j >= COLSIMAGE) break;
                if (y + j < void_col_max && y + j > void_col_min && x > void_row)
                {
                    boundary_cnt++;
                }
                else if (Target_Image.at<uchar>(x, y + j) > average)
                    boundary_cnt++;
            }

            if (boundary_cnt >= boundary_cnt_max)
            {
                boundary_cnt_max = boundary_cnt;
                //startingPoints.left = left_start[i];
                left_start_again.push_back(left_start[i]);
            }
        }
    }
    boundary_cnt_max = detection_width * 1.3;
    if (right_start.size() != 0)
    {
        for (int i = 0; i < right_start.size(); i++)
        {
            int boundary_cnt = 0;
            int x = right_start[i].x; int y = right_start[i].y;
            //��
            for (int j = 0; j < detection_width; j++)
            {
                if (y + j >= COLSIMAGE)
                {
                    break;
                }

                if (y + j < void_col_max && y + j > void_col_min && x > void_row)
                {
                    continue;
                }
                else if (Target_Image.at<uchar>(x, y + j) < average)
                {
                    boundary_cnt++;
                }
            }
            //��
            for (int j = 0; j < detection_width; j++)
            {
                if (y - j <= 0)
                {
                    break;
                }

                if (y - j < void_col_max && y - j > void_col_min && x > void_row)
                {
                    boundary_cnt++;;
                }
                else if (Target_Image.at<uchar>(x, y - j) > average)
                {
                    boundary_cnt++;
                }
            }
            if (boundary_cnt > boundary_cnt_max)
            {
                boundary_cnt_max = boundary_cnt;
                right_start_again.push_back(right_start[i]);
                //startingPoints.right = right_start[i];
            }
        }
        left_start.clear();
        right_start.clear();
        left_start = left_start_again;
        right_start = right_start_again;


        if (!left_start_again.empty() && !right_start_again.empty())
        {

            int min_dis = COLSIMAGE;
            for (int i = 0; i < left_start_again.size(); i++)
            {
                for (int j = 0; j < right_start_again.size(); j++)
                {
                    if ( right_start_again[j].y- left_start_again[i].y > 10)
                    {
                        int dis = abs(left_start_again[i].y - void_col_min) + abs(right_start_again[j].y - void_col_max);
                        if (dis < min_dis)
                        {
                            min_dis = dis;
                            startingPoints.left = left_start_again[i];
                            startingPoints.right = right_start_again[j];
                        }
                    }
                }

            }
            last_startingPoints = startingPoints;
        }
        else if (!left_start_again.empty())
        {
            int min_dis = COLSIMAGE;
            for (int i = 0; i < left_start_again.size(); i++)
            {
                if (abs(left_start_again[i].y - last_startingPoints.left.y) < min_dis)
                {
                    min_dis = abs(left_start_again[i].y - last_startingPoints.left.y);
                    startingPoints.left = left_start_again[i];
                }
            }
        }
        else if (!right_start_again.empty())
        {
            int min_dis = COLSIMAGE;
            for (int i = 0; i < right_start_again.size(); i++)
            {
                if (abs(right_start_again[i].y - last_startingPoints.right.y) < min_dis)
                {
                    min_dis = abs(right_start_again[i].y - last_startingPoints.right.y);
                    startingPoints.right = right_start_again[i];
                }
            }

        }
        
    }
    /*  last_startingPoints.left = startingPoints.left;
      last_startingPoints.right = startingPoints.right;*/
}

bool Tracking::isEdgePoint(Point& Point)
{
    int height = Target_Image.rows;
    int width = Target_Image.cols;
    // ����߽�����������Χ3��3���򳬳�ͼ��߽磬ֱ�ӷ����Ǳ�Ե
    if (Point.x < kernel || Point.x > height - 1 - kernel ||
        Point.y < kernel || Point.y > width - 1 - kernel) {
        return false;
    }
  /*  if (Point.y < void_col_max && Point.y > void_col_min && Point.x > void_row)
    {
        return false;
    }*/

    if (imageType == ImageType::Rgb) // ����RGBͼ��
    {
        // ��һȦ���أ�3��3�����ȥ�������أ�

        vector<int> firstRingPixels;
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                int new_x = Point.x + dx;
                int new_y = Point.y + dy;
                  
                firstRingPixels.push_back(Target_Image.at<Vec3b>(new_x, new_y)[1]);
            }
        }

        // �ڶ�Ȧ���أ�5��5��ȥ�м�3��3��
        vector<int> secondRingPixels;
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                if (abs(dx) <= 1 && abs(dy) <= 1) continue;
                int new_x = Point.x + dx;
                int new_y = Point.y + dy;

                secondRingPixels.push_back(Target_Image.at<Vec3b>(new_x, new_y)[1]);

            }
        }

        

        // �����һȦ���ص�ƽ�����ֵ��������������ĵ��ֵ�ľ���ֵ��������
        int centerValue = Target_Image.at<Vec3b>(Point.x, Point.y)[1];
        int sumFirstRingError = 0;
        for (int pixel : firstRingPixels) {
            sumFirstRingError += abs(pixel - centerValue);
        }
        double avgFirstRingError = (double)sumFirstRingError / firstRingPixels.size();

        // ����ڶ�Ȧ���ص�ƽ�����ֵ
        int sumSecondRingError = 0;
        for (int pixel : secondRingPixels) {
            sumSecondRingError += abs(pixel - centerValue);
        }
        double avgSecondRingError = (double)sumSecondRingError / secondRingPixels.size();

        // �趨Ȩ�ر�����ʾ��Ȩ�أ��ɸ���ʵ�ʵ�����
        double firstRingWeight = 0.7;
        double secondRingWeight = 0.3;
        double totalError = (firstRingWeight * avgFirstRingError +
            secondRingWeight * avgSecondRingError);

        // �����趨һ������ֵ���ж��Ƿ�Ϊ��Ե�㣬���Ը���ʵ���������
        double edgeThreshold = 30;
        return totalError > edgeThreshold;

    }

    if (imageType == ImageType::Binary) // ����binaryͼ��
    {
        const int centerValue = Target_Image.at<uchar>(Point.x, Point.y);
        int sumFirstRingError = 0; int cnt_first = 0;
        int sumSecondRingError = 0; int cnt_Second = 0;
       
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                if (dx == 0 && dy == 0) continue; // ������������

                int new_x = Point.x + dx;
                int new_y = Point.y + dy;

                // ����Ƿ��ڵ�һȦ��3x3����
                if (abs(dx) <= 1 && abs(dy) <= 1) {
                    cnt_first++;
                    sumFirstRingError += abs(Target_Image.at<uchar>(new_x, new_y) - centerValue);
                }
                // �������ڵڶ�Ȧ��5x5�����ȥ3x3����
                else {
                    cnt_Second++;
                    sumSecondRingError += abs(Target_Image.at<uchar>(new_x, new_y) - centerValue);
                }
            }
        }

        double avgFirstRingError = (double)sumFirstRingError / cnt_first;

        double avgSecondRingError = (double)sumSecondRingError / cnt_Second;

        // �趨Ȩ�ر�����ʾ��Ȩ�أ��ɸ���ʵ�ʵ�����
        double firstRingWeight = 0.7;
        double secondRingWeight = 0.3;
        double totalError = (firstRingWeight * avgFirstRingError +
            secondRingWeight * avgSecondRingError);

        // �����趨һ������ֵ���ж��Ƿ�Ϊ��Ե�㣬���Ը���ʵ���������
        double edgeThreshold = 34;
        return totalError > edgeThreshold;

    }

}

bool Tracking::isStartingPoints(Point point, bool left)
{
    return false;
}

//void Tracking::Extractedge()
//{
//    int x1, y1, x2, y2;
//    x1 = PointsEdgeLeft_BA[0].x; y1 = PointsEdgeLeft_BA[0].y;
//    Point PointTmp(x1, y1);
//    PointsEdgeLeft.push_back(PointTmp);
//    for (int i = 1; i < PointsEdgeLeft_BA.size(); i++)
//    {
//        x2 = PointsEdgeLeft_BA[i].x; y2 = PointsEdgeLeft_BA[i].y;
//        if (x2 == x1 - 1)
//        {
//            Point PointTmp(x2, y2);
//            PointsEdgeLeft.push_back(PointTmp);
//            x1 = x2; y1 = y2;
//        }
//
//    }//��ȡ���(��ͬ����ȡ:��)
//    x1 = PointsEdgeRight_BA[0].x; y1 = PointsEdgeRight_BA[0].y;
//    PointTmp.x = x1; PointTmp.y = y1;
//    PointsEdgeRight.push_back(PointTmp);
//    for (int i = 1; i < PointsEdgeRight_BA.size(); i++)
//    {
//        x2 = PointsEdgeRight_BA[i].x; y2 = PointsEdgeRight_BA[i].y;
//        if (x2 == x1 - 1)
//        {
//            Point PointTmp(x2, y2);
//            PointsEdgeRight.push_back(PointTmp);
//            x1 = x2; y1 = y2;
//        }
//
//    }//��ȡ���(��ͬ����ȡ:��)
//
//}
     
void Tracking::Edgecleaning()
{
    //�����ˣ�����һ��
    if (PointsEdgeLeft_BA.size() > 0 && PointsEdgeRight_BA.size() > 0)
    {
        if (PointsEdgeLeft_BA[PointsEdgeLeft_BA.size() - 1].x >= ROWSIMAGE - 10 || PointsEdgeRight_BA[PointsEdgeRight_BA.size() - 1].x >= ROWSIMAGE - 10)
        {
            Point p1 (0, 0);
            int Right_Left_x = ROWSIMAGE;
            int Left_Right_x = ROWSIMAGE;
            for (int i = 0; i < PointsEdgeRight_BA.size(); i++)
            {
                if (PointsEdgeRight_BA[i].y == kernel - 1)
                {
                    Right_Left_x = PointsEdgeRight_BA[i].x;
                    break;
                }
            }
            for (int i = 0; i < PointsEdgeLeft_BA.size(); i++)
            {
                if (PointsEdgeLeft_BA[i].y == COLSIMAGE - kernel)
                {
                    Left_Right_x = PointsEdgeLeft_BA[i].x;
                    break;
                }
            }
            if (Right_Left_x < Left_Right_x)
            {
                p1.x = Right_Left_x; p1.y = kernel - 1;
            }
            else
            {
                p1.x = Left_Right_x; p1.y = COLSIMAGE - kernel;
            }

            for (int i = 0; i < PointsEdgeRight_BA.size(); i++)
            {
                if (PointsEdgeRight_BA[i] == p1)
                {
                    PointsEdgeRight_BA.erase(PointsEdgeRight_BA.begin() + i, PointsEdgeRight_BA.end());
                    break;
                }
                   
            }
            for (int i = 0; i < PointsEdgeLeft_BA.size(); i++)
            {
                if (PointsEdgeLeft_BA[i] == p1)
                {
                    PointsEdgeLeft_BA.erase(PointsEdgeLeft_BA.begin() + i, PointsEdgeLeft_BA.end());
                    break;
                }

            }
        }


    }
}

void Tracking::drawImage(Mat& trackImage,double angle, Scene scene)
{
    for (int i = 0; i < yuan_PointsEdgeLeft_BA.size(); i++)
    {
        circle(trackImage, Point(yuan_PointsEdgeLeft_BA[i].y, yuan_PointsEdgeLeft_BA[i].x), 1,
            Scalar(0, 255, 0), -1); // ��ɫ��
    }
    for (int i = 0; i < yuan_PointsEdgeRight_BA.size(); i++)
    {
        circle(trackImage, Point(yuan_PointsEdgeRight_BA[i].y, yuan_PointsEdgeRight_BA[i].x), 1,
            Scalar(0, 255, 255), -1); // ��ɫ��
    }
    for (int i = 0; i < PointsEdgeLeft_BA.size(); i++)
    {
        circle(trackImage, Point(PointsEdgeLeft_BA[i].y, PointsEdgeLeft_BA[i].x), 1,
            Scalar(0, 255, 0), -1); // ��ɫ��
    }
    for (int i = 0; i < PointsEdgeRight_BA.size(); i++)
    {
        circle(trackImage, Point(PointsEdgeRight_BA[i].y, PointsEdgeRight_BA[i].x), 1,
            Scalar(0, 255, 255), -1); // ��ɫ��
    }
    for (int i = 0; i < midline.size(); i++)
    {
        circle(trackImage, Point(midline[i].y, midline[i].x), 1,
            Scalar(0, 0, 255), -1); // ɫ��
    }
    if (midline.size() > 1)
    {
        circle(trackImage, Point(midline[midline.size() - 1].y, midline[midline.size() - 1].x), 5,
            Scalar(0, 0, 255), -1); // ɫ��
    }
    rectangle(trackImage,Point(void_col_min, void_row),Point(void_col_max, void_row ),Scalar(0, 0, 0),1);
    rectangle(trackImage, Point(void_col_min, void_row), Point(void_col_min, ROWSIMAGE - 1), Scalar(0, 0, 0), 1);
    rectangle(trackImage, Point(void_col_max, void_row), Point(void_col_max, ROWSIMAGE - 1), Scalar(0, 0, 0), 1);



    circle(trackImage, Point(startingPoints.left.y, startingPoints.left.x), 5 ,Scalar(0, 0, 255), -1); // ��ɫ��
    circle(trackImage, Point(startingPoints.right.y, startingPoints.right.x), 5 ,Scalar(0, 0, 255), -1); // ��ɫ��
    putText(trackImage, "angle: " + std::to_string(angle), Point(100, 30),FONT_HERSHEY_TRIPLEX, 0.5,Scalar(0, 0, 0), 1, CV_AA);
    switch (scene) {
    case Scene::NormalScene:
        break;
    case Scene::RingScene:              // [ ���� ]
        circle(trackImage, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 20, Scalar(40, 120, 250), -1);
        putText(trackImage, "R", Point(COLSIMAGE / 2 - 10, ROWSIMAGE / 2 + 10), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2);
        break;
    case Scene::CrossScene:                  // [ ʮ���� ]
        circle(trackImage, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 20, Scalar(40, 120, 250), -1);
        putText(trackImage, "+", Point(COLSIMAGE / 2 - 10, ROWSIMAGE / 2 + 10), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2);
        break;
    case Scene::ParkingScene:          // [ ���ͣ���� ]
        circle(trackImage, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 20, Scalar(40, 120, 250), -1);
        putText(trackImage, "P", Point(COLSIMAGE / 2 - 10, ROWSIMAGE / 2 + 10), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2);
        break;
    case Scene::ObstacleScene:    //[ �ϰ��� ]
        circle(trackImage, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 20, Scalar(40, 120, 250), -1);
        putText(trackImage, "X", Point(COLSIMAGE / 2 - 10, ROWSIMAGE / 2 + 10), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2);
        break;

    default: // �����·������������·���滮
        break;
    }


   /* putText(trackImage, to_string(validRowsRight) + " " + to_string(stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
        FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);
    putText(trackImage, to_string(validRowsLeft) + " " + to_string(stdevLeft), Point(20, ROWSIMAGE - 50),
        FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);*/
}

int Tracking::re_rowstart(void)
{
    return rowStart;
}

void Tracking::PointsEdge_extract(void)
{
    //if (PointsEdgeLeft_BA.size() != 0)
    //{
    //    int x1, y1, x2, y2;
    //    x1 = PointsEdgeLeft_BA[0].x; y1 = PointsEdgeLeft_BA[0].y;
    //    Point pointTmp(x1, y1);
    //    PointsEdgeLeft.push_back(pointTmp);
    //    for (int i = 0; i < PointsEdgeLeft_BA.size(); i++)
    //    {
    //        x2 = PointsEdgeLeft_BA[i].x; y2 = PointsEdgeLeft_BA[i].y;
    //        if (x2 == x1 - 1)
    //        {
    //            Point pointTmp(x2, y2);
    //            PointsEdgeLeft.push_back(pointTmp);
    //            x1 = x2; y1 = y2;
    //        }

    //    }

    //}
    //if (PointsEdgeRight_BA.size() != 0)
    //{
    //    int x1, y1, x2, y2;
    //    x1 = PointsEdgeRight_BA[0].x; y1 = PointsEdgeRight_BA[0].y;
    //    Point pointTmp(x1, y1);
    //    PointsEdgeRight.push_back(pointTmp);
    //    for (int i = 0; i < PointsEdgeRight_BA.size(); i++)
    //    {
    //        x2 = PointsEdgeRight_BA[i].x; y2 = PointsEdgeRight_BA[i].y;
    //        if (x2 == x1 - 1)
    //        {
    //            Point pointTmp(x2, y2);
    //            PointsEdgeRight.push_back(pointTmp);
    //            x1 = x2; y1 = y2;
    //        }

    //    }

    //}//������ȡ���(��ͬ����ȡ)

}