#include "../include/common.h"

// 计算两个向量的夹角
double calculateAngle(const Point& a, const Point& b)
{
    double dot_prod = a.x * b.x + a.y * b.y;
    double len1 = std::sqrt(a.x * a.x + a.y * a.y);
    double len2 = std::sqrt(b.x * b.x + b.y * b.y);

    double angle = std::acos(dot_prod / (len1 * len2));

    double cross_prod_z = a.x * b.y - a.y * b.x;

    if (cross_prod_z < 0) {
        angle = 2* 3.14159265358979323846 - angle;
    }

    return angle;
}

Point Cornerdetection(const Point point,Mat m_H)
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

// 输入两个点，输出垂直
double Orthogonalvector(Point& a, Point& b)
{
    Vec2d C;
    C[0] = (a.y - b.y);
    C[1] = (a.x - b.x);

    if (C[0] == 0)
    {
        return  1000;// 水平,返回竖直
    }
    if (C[1] == 0)
    {
        return 0;// 竖直，返回水平
    }
    double k = C[0] / C[1];    
    return -1.0 / k;
}


bool isStraightLine(const std::vector<cv::Point>& points, double max_error) {
    const size_t n = points.size();
    if (n < 30) return false; //数量不足

    // 1. 拟合直线（OpenCV的fitLine返回值格式为[vx, vy, x0, y0]）
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);

    // 2. 提取直线参数
    const float vx = line[0], vy = line[1];  // 单位方向向量
    const float px = line[2], py = line[3];  // 直线上一点

    // 3. 计算法向量（无需归一化，fitLine已返回单位向量）
    const float nx = -vy;  // 法向量x
    const float ny = vx;   // 法向量y

    // 4. 采样检查（至少检查首尾点）
    const size_t sample_step = std::max<size_t>(1, n / 5);
    for (size_t i = 0; i < n; i += sample_step) {
        const auto& pt = points[i];
        float dx = static_cast<float>(pt.x) - px;
        float dy = static_cast<float>(pt.y) - py;
        float dist = std::abs(nx * dx + ny * dy);
        if (dist > max_error) return false;
    }
    return true;
}

void framerate()
{
    static auto preTime = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch())
        .count();
    auto startTime = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch())
        .count();
    cout << "run frame time : " << startTime - preTime << "ms" << endl;
    preTime = startTime;

}

double sigma(vector<Point> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    double my_sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        my_sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    my_sigma /= (double)vec.size();
    return my_sigma;
}

string img_path(string path)
{

    size_t startPos = path.find_last_of('/') + 1;
    size_t endPos = path.find_last_of('.');
    std::string numberStr = path.substr(startPos, endPos - startPos);

    int number = std::stoi(numberStr);
    number = number + 1;
    std::string newNumberStr = std::to_string(number);
    path.replace(startPos, endPos - startPos, newNumberStr);

    return path;
}