
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

using namespace std;
using namespace cv;
static double computeReprojectionErrors(
	const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs
)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();

		totalErr += err*err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}

int main(int argc, char** argv)
{
	vector<string> files_left;
	vector<string> files_right;
	glob("left0", files_left);
	glob("right0", files_right);
	// 定义变量
	vector<vector<Point2f>> image_leftPoints,image_rightPoints;//像点
	vector<vector<Point3f>> objectPoints;//物点
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);//进行亚像素精度的调整，获得亚像素级别的角点坐标
	int numCornersHor = 6;//heigh
	int numCornersVer = 8;//width
	int numSquares = 10;//单位mm
	Mat gray_l, gray_r;
	Mat image_l, image_r;
	vector<Point3f> obj;
	//存放每张图的角点坐标，并存入obj中(物点)
	for (int i = 0; i < numCornersHor; i++)
		for (int j = 0; j < numCornersVer; j++)
			obj.push_back(Point3f((float)j * numSquares, (float)i * numSquares, 0));
	
	Size s1,s2;
	//像点
	for (int i = 0; i < files_left.size(); i++) {
		printf("image file : %s \n", files_left[i].c_str());
		 image_l = imread(files_left[i]);
		printf("image file : %s \n", files_right[i].c_str());
		 image_r = imread(files_right[i]);
		s1 = image_l.size();
		s2 = image_r.size();
		
		cvtColor(image_l, gray_l, COLOR_BGR2GRAY);//转灰度
		cvtColor(image_r, gray_r, COLOR_BGR2GRAY);//转灰度
		vector<Point2f> corners_r;
		vector<Point2f> corners_l;
		bool ret1 = findChessboardCorners(gray_r, Size(8, 6), corners_r, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);//该函数的功能就是判断图像内是否包含完整的棋盘图，若能检测完全，就把他们的角点坐标(从上到下，从左到右)记录，并返回true，否则为false，CALIB_CB_FILTER_QUADS用于去除检测到的错误方块。
		bool ret2 = findChessboardCorners(gray_l, Size(8, 6), corners_l, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		if (ret1)
		{
			cornerSubPix(gray_l, corners_l, Size(8, 8), Size(-1, -1), criteria);//用于发现亚像素精度的角点位置
			drawChessboardCorners(image_l, Size(8, 6), corners_l, ret1);//将每一个角点出做标记，此为物点对应的像点坐标	
			//imshow("calibration-demo1", image_l);
			//waitKey(500);
		}
		if (ret2)
		{
			cornerSubPix(gray_r, corners_r, Size(8, 8), Size(-1, -1), criteria);//用于发现亚像素精度的角点位置
			drawChessboardCorners(image_r, Size(8, 6), corners_r, ret2);//将每一个角点出做标记，此为物点对应的像点坐标
		//将角点坐标存入imagePoints中，此为像点坐标	
			//imshow("calibration-demo", image_r);
			//waitKey(500);
		}
		if (ret1&&ret2)
		{
			image_rightPoints.push_back(corners_r);
			image_leftPoints.push_back(corners_l);//将角点坐标存入imagePoints中，此为像点坐标
			objectPoints.push_back(obj);
		}

	}

		//计算内参与畸变系数
		Mat intrinsic_left = Mat(3, 3, CV_32FC1);
		
		Mat distCoeffs_left;//畸变矩阵
		vector<Mat> rvecs_l;//旋转向量R
		vector<Mat> tvecs_l;//平移向量T
	
						  //内参矩阵
		intrinsic_left.ptr<float>(0)[0] = 1;
		intrinsic_left.ptr<float>(1)[1] = 1;
		calibrateCamera(objectPoints, image_leftPoints, s1, intrinsic_left, distCoeffs_left, rvecs_l, tvecs_l);
		//计算内参与畸变系数
		Mat intrinsic_right = Mat(3, 3, CV_32FC1);
		Mat distCoeffs_right;//畸变矩阵
		vector<Mat> rvecs_r;//旋转向量R
		vector<Mat> tvecs_r;//平移向量T
		Mat R_total;//旋转向量R
		Vec3d T_total;//平移向量T
		Mat E ;//本质矩阵
		Mat F ;//基本矩阵
		intrinsic_right.ptr<float>(0)[0] = 1;
		intrinsic_right.ptr<float>(1)[1] = 1;
		calibrateCamera(objectPoints, image_rightPoints, s2, intrinsic_right, distCoeffs_right, rvecs_r, tvecs_r);
		FileStorage fs("out_calibration.yml", FileStorage::WRITE);//存储标定结果，这里可以选择自己的存放路径
		fs << "intrinsic_left" << intrinsic_left;//存放内参矩阵
		fs << "distCoeffs_left" << distCoeffs_left;//存放畸变矩阵
		fs << "board_width" << 8;//存放标定板长度信息
		fs << "board_height" << 6;//存放标定板宽度信息
		fs << "square_size" << 0.01;//存放标定板格子尺寸信息
		fs << "R_left" << rvecs_l;//左相机旋转矩阵
		fs << "T_left" << tvecs_l;//左相机平移矩阵
		fs << "intrinsic_right" << intrinsic_right;//存放内参矩阵
		fs << "distCoeffs_right" << distCoeffs_right;//存放畸变矩阵
		fs << "R_right" << rvecs_r;//右相机旋转矩阵
		fs << "T_right" << tvecs_r;//右相机平移矩阵
	
		printf("Done Calibration\n");
		Mat R_L;//由R_total拆分的左相机旋转量
		Mat R_R;//由R_total拆分的右相机旋转量
		Mat P1;
		Mat P2;
		Mat Q;
		Rect validROIL, validROIR;
		//计算相机之间的平移旋转矩阵
		cv::stereoCalibrate(objectPoints, image_leftPoints,  image_rightPoints, intrinsic_left, distCoeffs_left, intrinsic_right, distCoeffs_right, s1, R_total, T_total, E, F);
		printf("Starting Rectification\n");
		stereoRectify(intrinsic_left, distCoeffs_left, intrinsic_right, distCoeffs_right, s1, R_total, T_total, R_L, R_R, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, s1, &validROIL, &validROIR);
		fs << "R_total" << R_total;//全局旋转矩阵
		fs << "T_total" << T_total;//全局平移矩阵
		fs << "R_left" << R_L;//存放内参矩阵
		fs << "R_right" << R_R;//存放畸变矩阵
		fs << "P1" << P1;//存放标定板长度信息
		fs << "P2" << P2;//存放标定板宽度信息
		fs << "Q" << Q;//存放标定板格子尺寸信息
		fs << "E" << E;//存放标定板格子尺寸信息
		fs << "F" << F;//存放标定板格子尺寸信息
		cout << "left_Calibration error: " << computeReprojectionErrors(objectPoints, image_leftPoints, rvecs_l, tvecs_l, intrinsic_left, distCoeffs_left) << endl;//输出投影的误差
		cout << "right_Calibration error: " << computeReprojectionErrors(objectPoints, image_rightPoints, rvecs_r, tvecs_r, intrinsic_right, distCoeffs_right) << endl;//输出投影的误差
		printf("Done Rectification\n");

		//根据stereoRectify计算出来的R和P来计算图像的映射表 mapx,mapy
		//mapx,mapy这两个映射表接下来可以给remap（）函数调用，校正图像(使图像共勉并且行对准)
		//ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的相机矩阵，
		Mat mapLx, mapLy, mapRx, mapRy;
		initUndistortRectifyMap(intrinsic_left, distCoeffs_left, R_L, P1, s1, CV_16SC2, mapLx, mapLy);
		initUndistortRectifyMap(intrinsic_right, distCoeffs_right, R_R, P2, s1, CV_16SC2, mapRx, mapRy);
	     ////https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a

		printf("get map\n");
		Mat rectifyImageL2, rectifyImageR2;
		remap(image_l, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
		remap(image_r, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);
		printf("rectify done\n");
		//cout << "按Q2退出 ..." << endl;
		imshow("rectifyImageL", rectifyImageL2);
		imshow("rectifyImageR", rectifyImageR2);
		imwrite("rectifyImageL2.png", rectifyImageL2);
		imwrite("rectifyImageR2.png", rectifyImageR2);
		waitKey(10000);
		Mat canvas;
		double sf;
		int w, h;
		sf = 600./MAX(s1.width, s1.height);
		w = cvRound(s1.width * sf);
		h = cvRound(s1.height * sf);
		canvas.create(h, w * 2, CV_8UC3);

		/*左图像画到画布上*/
		Mat canvasPart = canvas(Rect(w * 0, 0, w, h));
		resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);//把图像缩放到跟canvasPart一样大小  
		Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),//获得被截取的区域    
			cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
		rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8); //画上一个矩形  

		/*右图像画到画布上*/
		canvasPart = canvas(Rect(w, 0, w, h)); //获得画布的另一部分  
		resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
		Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
			cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
		rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

		cout << "Painted ImageR" << endl;

		/*画上对应的线条*/
		for (int i = 0; i < canvas.rows; i += 16)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

		imshow("rectified", canvas);

		cout << "wait key" << endl;
		waitKey(0);
		
		return 0;
	}
