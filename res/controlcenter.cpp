#include "../include/controlcenter.h"



double ControlCenterCal::Feedback_extraction(Tracking& tracking, PerspectiveMapping& mapping,bool dir)
{
	//边界转换
	//Edgehandling(tracking, mapping);



	//中线提取
	midlineextraction(mapping, dir);//dir=1为左边
	
	//中线映射
	midline_track(tracking, mapping);

	//角度计算
	return pure_pursuit(tracking, mapping);

}


//不在这里用了
void ControlCenterCal::Edgehandling(Tracking& tracking, PerspectiveMapping &mapping)
{
	///*PointsEdgeLeft.clear();
	//PointsEdgeRight.clear();*/
	//centerEdge.clear();
	//
	////tracking.PointsEdge_extract();

	//int length = 2;
	//for (int i = 0; i < tracking.PointsEdgeLeft_BA.size() - length; i+= length)
	//{
	//	if (tracking.PointsEdgeLeft_BA[i].y <= kernel)
	//		continue;
	//	else
	//	{
	//		Point pointtemp = mapping.Cornerdetection(tracking.PointsEdgeLeft_BA[i]);

	//		PointsEdgeLeft.push_back(pointtemp);
	//		//PointsEdgeLeft.push_back(pointtemp);
	//	}
	//}
	//for (int i = 0; i < tracking.PointsEdgeRight_BA.size() - length; i+=length)
	//{
	//	if (tracking.PointsEdgeRight_BA[i].y >= COLSIMAGE - kernel - 1)
	//		continue;
	//	else
	//	{
	//		Point pointtemp = mapping.Cornerdetection(tracking.PointsEdgeRight_BA[i]);
	//		PointsEdgeRight.push_back(pointtemp);
	//		//PointsEdgeRight.push_back(pointtemp);
	//	}
	//}

	//vector<Point> tempPointsEdgeLeft;
	//
	//for (int i = 0; i < tracking.PointsEdgeLeft_BA.size(); i++)
	//{
	//	if (tracking.PointsEdgeLeft_BA[i].y <= kernel)
	//		continue;
	//	else
	//	{
	//		Point pointtemp = mapping.Cornerdetection(tracking.PointsEdgeLeft_BA[i]);

	//		tempPointsEdgeLeft.push_back(pointtemp);
	//		//PointsEdgeLeft.push_back(pointtemp);
	//	}
	//}
	//
	//vector<Point> tempPointsEdgeRight;
	//for (int i = 0; i < tracking.PointsEdgeRight_BA.size(); i++)
	//{
	//	if (tracking.PointsEdgeRight_BA[i].y >= COLSIMAGE - kernel - 1)
	//		continue;
	//	else
	//	{
	//		Point pointtemp = mapping.Cornerdetection(tracking.PointsEdgeRight_BA[i]);
	//		tempPointsEdgeRight.push_back(pointtemp);
	//		//PointsEdgeRight.push_back(pointtemp);
	//	}
	//}

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

void ControlCenterCal::midlineextraction(PerspectiveMapping& mapping,bool left)
{

	centerEdge.clear();
	double Pixel_Pitch = mapping.PixelPitch();
	double point_Distance = 22.5 / Pixel_Pitch;
	int dis = 1;



	if (left == true && mapping.PointsEdgeLeft.size() > dis + 1)
	{
		vector<Point> PointsEdgeLeft_m = smoothTrajectory(mapping.PointsEdgeLeft, 25, 9.0);

		
		centerEdge.reserve(PointsEdgeLeft_m.size() / dis);
		for (int i = dis; i < PointsEdgeLeft_m.size() - 2 - dis; i += dis)
		{
			Point vec = PointsEdgeLeft_m[i + dis] - PointsEdgeLeft_m[i - dis];
			Point shiftVec(-vec.y, vec.x);
			double next_y, next_x;

			// 计算法向量单位化的比例系数
			double length = sqrt(shiftVec.x * shiftVec.x + shiftVec.y * shiftVec.y);
			if (length < 1e-6) {
				// 如果法向量长度过小，则跳过此点或进行其他处理
				continue;
			}

			double k = point_Distance / length;
			next_x = PointsEdgeLeft_m[i].x + k * shiftVec.x;
			next_y = PointsEdgeLeft_m[i].y + k * shiftVec.y;

			Point pointemp(next_x, next_y);
			centerEdge.push_back(pointemp);
		}
	}

	if (left == false && mapping.PointsEdgeRight.size() > dis + 1)
	{
		vector<Point> PointsEdgeRight_m = smoothTrajectory(mapping.PointsEdgeRight, 25, 9.0);

		centerEdge.reserve(PointsEdgeRight_m.size() / dis);
		for (int i = dis; i < PointsEdgeRight_m.size() - 2 - dis; i += dis)
		{
			Point vec = PointsEdgeRight_m[i + dis] - PointsEdgeRight_m[i - dis];
			Point shiftVec(vec.y, -vec.x);
			double next_y, next_x;
			double k = point_Distance / sqrt(shiftVec.x * shiftVec.x + shiftVec.y * shiftVec.y);
			next_x = PointsEdgeRight_m[i].x + k * shiftVec.x;
			next_y = PointsEdgeRight_m[i].y + k * shiftVec.y;

			Point pointemp(next_x, next_y);
			centerEdge.push_back(pointemp);
		}
	}
	centerEdge = smoothTrajectory(centerEdge,13,4.0);
	 
	 
	// 控制率计算
	//if (centerEdge.size() > 20)
	//{
	//	vector<Point> centerV;
	//	int filt = centerEdge.size() / 5;
	//	for (int i = filt; i < centerEdge.size() - filt; i++) // 过滤中心点集前后1/5的诱导性
	//	{
	//		centerV.push_back(centerEdge[i]);
	//	}
	//	sigmaCenter = sigma(centerV);
	//}
	//else
	//	sigmaCenter = 1000;
}

void ControlCenterCal::midline_track(Tracking& tracking,PerspectiveMapping& mapping)
{
	std::vector<cv::Point2f> centerEdgeFloat(centerEdge.begin(), centerEdge.end());

	if (centerEdge.size() > 0)
	{
		tracking.midline.reserve(centerEdge.size());
		for (int i = 0; i < centerEdge.size(); i++)
		{
			Point pointemp = mapping.InverseCornerDetection(centerEdge[i]);
			tracking.midline.push_back(pointemp);
		}
	}
}


void ControlCenterCal::Deviation_calculation()
{


}

void ControlCenterCal::ControlCenterCal::drawImage(Mat& trackImage)
{   
	for (int i = 0; i < centerEdge.size()  ; i++)
	{
		circle(trackImage, Point(centerEdge[i].x, centerEdge[i].y), 1,
			Scalar(255, 0, 255), -1); // 
	}
	
	if(!centerEdge.empty())
		circle(trackImage, Point(centerEdge[centerEdge.size() - 1].x, centerEdge[centerEdge.size() - 1].y), 4,
			Scalar(0, 255, 255), -1); // 



    //circle(trackImage, Point(startingPoints.left.y, startingPoints.left.x), 5, Scalar(0, 0, 255), -1); // 红色点
    //circle(trackImage, Point(startingPoints.right.y, startingPoints.right.x), 5, Scalar(0, 0, 255), -1); // 红色点

    /* putText(trackImage, to_string(validRowsRight) + " " + to_string(stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
         FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);
     putText(trackImage, to_string(validRowsLeft) + " " + to_string(stdevLeft), Point(20, ROWSIMAGE - 50),
         FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);*/
}


std::vector<cv::Point> ControlCenterCal::smoothTrajectory(const std::vector<cv::Point>& trajectory,int window_size,double sigma)
{
	std::vector<cv::Point> smoothed;
	if (trajectory.empty() || window_size < 1) return smoothed;

	const int n = trajectory.size();
	smoothed.reserve(n);

	// 高斯权重计算（替代三角形权重）
	std::vector<double> weights;
	weights.reserve(window_size);
	double sum_weights = 0.0;
	for (int i = 0; i < window_size; ++i) {
		double weight = exp(-(i * i) / (2 * sigma * sigma)); // 高斯核
		weights.push_back(weight);
		sum_weights += weight;
	}

	// 边界镜像填充（避免边界失真）
	auto get_mirrored_point = [&](int idx) {
		if (idx < 0) return trajectory[0];
		if (idx >= n) return trajectory[n - 1];
		return trajectory[idx];
		};

	// 滑动窗口加权平均
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


double ControlCenterCal::pure_pursuit(Tracking& tracking, PerspectiveMapping& mapping)
{
	if (centerEdge.size() < 2)
	{
		cout << "centerEdge.size() < 2" << endl;
		return 0;
	}
	//current_pose:后轮中点（大概）
	double Pixel_Pitch = mapping.PixelPitch();
	Point p1(ROWSIMAGE - 1, COLSIMAGE / 2);
	Point current_pose = mapping.Cornerdetection(p1);
	current_pose.y = current_pose.y + 14 / Pixel_Pitch;
	//wheel_base:轴距
	double wheel_base = 20 / Pixel_Pitch;

	//目标点确定
	Point aim_point(0, 0);
	for (int i = 0; i < centerEdge.size(); i++)
	{
		double dis = sqrt((centerEdge[i].x - current_pose.x) * (centerEdge[i].x - current_pose.x) + (centerEdge[i].y - current_pose.y) * (centerEdge[i].y - current_pose.y));
		//70是预瞄距离(比较近)

		if (dis * Pixel_Pitch >  60)
		{
			aim_point = centerEdge[i];
			Point pointemp = mapping.InverseCornerDetection(aim_point);
			centerEdge.push_back(aim_point);
			tracking.midline.push_back(pointemp);
			break;
		}
	}	
	if (aim_point.x == 0 && aim_point.y == 0)
	{
		aim_point = centerEdge[centerEdge.size() - 1];
		
	}
	//计算tmp_steering:转向角
	double dx = aim_point.x - current_pose.x;
	double dy = aim_point.y - current_pose.y;

	double lookahead_distance = std::sqrt(dx * dx + dy * dy);

	double target_curvature = dx / lookahead_distance;

	double tmp_steering = atan( 2 * target_curvature * wheel_base / lookahead_distance) * 180.0 / 3.1415926;

	return tmp_steering;

}

