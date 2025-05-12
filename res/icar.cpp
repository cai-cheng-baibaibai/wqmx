#include<iostream>
#include "../include/tracking.h"
#include "../include/mapping.h"
#include "../include/motion.h"
#include "../include/ring.h"
#include "../include/controlcenter.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../include/common.h"
#include "../include/crossroad.h"
#include "../include/parking.h"
#include "conio.h"

using namespace std;
using namespace cv;
using std::cout;

int main()
{
	Tracking tracking;
	PerspectiveMapping mapping;
	ControlCenterCal controlCenterCal;              // �������ļ���
	Crossroad crossroad;
	Parking parking;          
	Motion motion;
	Ring ring;

	// ��ʼ������
	Scene scene = Scene::NormalScene;     // ��ʼ�������������·
	Scene sceneLast = Scene::NormalScene; // ��¼��һ�γ���״̬

	//VideoCapture capture(0);
	
	Mat imgResize, hsvImage,img;
	Mat gray_image(ROWSIMAGE, COLSIMAGE, CV_8UC1);
	/*img = cv::imread("../img/0.jpg");*/
	//"resizeimg/1437.jpg"
	string where = "resizeimg/1.jpg";
	string send_trackimg_address = "hamimg1/0.jpg";
	double angle;
	tracking.rowStart = ROWSIMAGE - 30;//195
	//tracking.rowStart = 50;

	tracking.m_H = mapping.get_m_H();

	while (true)
	{
		framerate();
		//capture.read(img);

		img = cv::imread(where);
		
		//where = img_path(where);

		send_trackimg_address = img_path(send_trackimg_address);
		if (img.empty()) {
			std::cerr << "Error loading image" << std::endl;
			return -1;
		}

		//��С����
		resize(img, imgResize, Size(COLSIMAGE, ROWSIMAGE), 0, 0, INTER_NEAREST);
		//ͼƬתΪ�Ҷ�
		cvtColor(imgResize, hsvImage, cv::COLOR_BGR2GRAY);//COLOR_BGR2HSV
		//����ʶ��
		tracking.trackRecognition(hsvImage, tracking.rowStart,mapping.get_m_H(),mapping.PixelPitch());
		//�߽�͸�ӣ��ǵ�Ѱ��
		mapping.Edgehandling(tracking);

		//��ʼ����Ԫ�ز���
		if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
			motion.params.cross) {
			if (crossroad.crossRecognition(tracking, mapping))
				scene = Scene::CrossScene;
			else
				scene = Scene::NormalScene;
		}
		if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
			motion.params.ring) {
			if (ring.process(tracking,mapping))
				scene = Scene::RingScene;
			else
				scene = Scene::NormalScene;
		}
		if ((scene == Scene::NormalScene || scene == Scene::ParkingScene) &&
			motion.params.parking) {
			if (parking.process(tracking, hsvImage, mapping))  // �����ֵ��ͼ������ٴ���
				scene = Scene::ParkingScene;
			else
				scene = Scene::NormalScene;
		}


		//direction = 0:Ĭ�����ұ�
		if (scene == Scene::NormalScene)
		{
			if(!tracking.PointsEdgeRight_BA.empty())
				mapping.direction = 0;
			else
			{
				mapping.direction = 1;
			}
		}

		//�Ƕȼ��㣬ir=1Ϊ��ߣ�dirҪ�ģ�
		//mapping.direction = 1;
		angle = controlCenterCal.Feedback_extraction(tracking, mapping, mapping.direction);


		Mat blackImage(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
		 
		
		//for (int y = 0; y < ROWSIMAGE; y++)
		//{
		//	for (int x = 0; x < COLSIMAGE; x++) {
		//		// ��������ֵΪ 100            
		//		Point Pointtemp(y, x);
		//		if (tracking.isEdgePoint(Pointtemp) == 1)
		//		{
		//			gray_image.at<uchar>(y, x) = 0; //��ɫ
		//		}
		//		else if (tracking.isEdgePoint(Pointtemp) == 0)
		//		{
		//			gray_image.at<uchar>(y, x) = 255; //��ɫ
		//		}

		//	}
		//}

		//imwrite(send_trackimg_address, imgResize);
		tracking.drawImage(imgResize,angle,scene);
		mapping.drawImage(blackImage);
		controlCenterCal.drawImage(blackImage);

		//circle(imgResize, Point(crossroad.pt_Left.y, crossroad.pt_Left.x), 5,
		//	Scalar(0, 0, 0), -1); // ��ɫ��

		//circle(imgResize, Point(crossroad.pt_Right.y, crossroad.pt_Right.x), 5,
		//	Scalar(0, 0, 0), -1); // ��ɫ��
		
		imshow("gray_image", gray_image);
		moveWindow("gray_image", 100, 100);
		imshow("blackImage", blackImage);
	
		imshow("imresize", imgResize);

		cout << where << endl;

		waitKey(1);

		if (_kbhit())
		{
			int ch = _getch();
			std::cout << "Key pressed: " << ch << std::endl;
			if(ch == 13)
			where = img_path(where);
		}


	}

	

	


}
