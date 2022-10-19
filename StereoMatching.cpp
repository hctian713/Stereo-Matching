// StereoMatching.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2\imgproc\types_c.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\calib3d\calib3d.hpp>

using namespace std;
using namespace cv;

enum { STEREO_SGBM = 0, STEREO_HH = 1, STEREO_3WAY = 2, STEREO_HH4 = 3 };
void calDispWithBM(Mat left, Mat right, Mat& imgDisparity8U)
{
	cv::cvtColor(left, left, CV_BGR2GRAY);
	cv::cvtColor(right, right, CV_BGR2GRAY);

	Mat imgDisparity16S = Mat(left.rows, left.cols, CV_16S);

	//--Call the constructor for StereoBM
	cv::Size imgSize = left.size();
	int numberOfDisparities = ((imgSize.width / 8) + 15) & -16;
	cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);

	//--Calculate the disparity image

	/*
	左右视图的有效像素区域，一般由双目校正阶段的 cvStereoRectify 函数传递，也可以自行设定。
	一旦在状态参数中设定了 roi1 和 roi2，OpenCV 会通过cvGetValidDisparityROI 函数计算出视差图的有效区域，在有效区域外的视差值将被清零。
	*/
	cv::Rect roi1, roi2;
	bm->setROI1(roi1);
	bm->setROI2(roi2);

	//==预处理滤波参数
	/*
	预处理滤波器的类型，主要是用于降低亮度失真（photometric distortions）、消除噪声和增强纹理等,
	有两种可选类型：CV_STEREO_BM_NORMALIZED_RESPONSE（归一化响应） 或者 CV_STEREO_BM_XSOBEL（水平方向Sobel算子，默认类型）,
	该参数为 int 型;
	*/
	bm->setPreFilterType(StereoBM::PREFILTER_NORMALIZED_RESPONSE);

	/*预处理滤波器窗口大小，容许范围是[5,255]，一般应该在 5x5..21x21 之间，参数必须为奇数值, int 型*/
	bm->setPreFilterSize(9);

	/*预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围：1 - 31,int 型*/
	bm->setPreFilterCap(31);

	//==SAD 参数
	/*SAD窗口大小，容许范围是[5,255]，一般应该在 5x5 至 21x21 之间，参数必须是奇数，int 型*/
	bm->setBlockSize(9);

	/*最小视差，默认值为 0, 可以是负值，int 型*/
	bm->setMinDisparity(0);

	/*视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍，int 型*/
	bm->setNumDisparities(numberOfDisparities);

	//==后处理参数
	/*
	低纹理区域的判断阈值:
	如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，则该窗口对应的像素点的视差值为 0
	（That is, if the sum of absolute values of x-derivatives computed over SADWindowSize by SADWindowSize
	pixel neighborhood is smaller than the parameter, no disparity is computed at the pixel），
	该参数不能为负值，int 型;
	*/
	bm->setTextureThreshold(10);

	/*
	视差唯一性百分比:
	视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，
	否则该像素点的视差为 0 （the minimum margin in percents between the best (minimum) cost function value and the second best value to accept
	the computed disparity, that is, accept the computed disparity d^ only if SAD(d) >= SAD(d^) x (1 + uniquenessRatio/100.) for any d != d*+/-1 within the search range ），
	该参数不能为负值，一般5-15左右的值比较合适，int 型*/
	bm->setUniquenessRatio(15);

	/*检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查，int 型*/
	bm->setSpeckleWindowSize(100);

	/*视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，int 型*/
	bm->setSpeckleRange(32);

	/*
	左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。
	超过该阈值的视差值将被清零。该参数默认为 -1，即不执行左右视差检查。int 型。
	注意在程序调试阶段最好保持该值为 -1，以便查看不同视差窗口生成的视差效果。
	*/
	bm->setDisp12MaxDiff(1);

	/*计算视差*/
	int64 t = getTickCount();
	bm->compute(left, right, imgDisparity16S);
	t = getTickCount() - t;
	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;

	//--Display it as a CV_8UC1 image：16位有符号转为8位无符号
	imgDisparity16S.convertTo(imgDisparity8U, CV_8U, 255 / (numberOfDisparities * 16.));
	imshow("disparity", imgDisparity8U);
	imwrite("test.png", imgDisparity8U);
	waitKey();
}
void calDispWithSGBM(Mat left, Mat right, Mat& imgDisparity8U) 
{
	//设置参数
	cv::Size imgSize = left.size();
	int numberOfDisparities = ((imgSize.width / 8) + 15) & -16;
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);

	sgbm->setPreFilterCap(63);

	int SADWindowSize = 9;
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);

	int cn = left.channels();
	sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
	sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);

	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	//选择算法模式
	int alg = STEREO_SGBM;
	if (alg == STEREO_HH)
		sgbm->setMode(cv::StereoSGBM::MODE_HH);
	else if (alg == STEREO_SGBM)
		sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
	else if (alg == STEREO_3WAY)
		sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
	else if (alg == STEREO_HH4)
		sgbm->setMode(cv::StereoSGBM::MODE_HH4);

	Mat imgDisparity16S = Mat(left.rows, left.cols, CV_16S);

	//计算并显示视差
	int64 t = getTickCount();
	sgbm->compute(left, right, imgDisparity16S);
	t = getTickCount() - t;
	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
	imgDisparity16S.convertTo(imgDisparity8U, CV_8U, 255 / (numberOfDisparities * 16.));   //将16位符号整形的视差矩阵转换为8位无符号整形矩阵/

	imshow("disparity", imgDisparity8U);
	imwrite("4SGBM.png", imgDisparity8U);
	waitKey();


}

int main()
{

	//--读取图像
	Mat imgL = imread("000000_10.png");
	Mat imgR = imread("000000_11.png");
	if (imgL.empty() || imgR.empty()) {
		std::cout << "could not load image...\n";
		return -1;
	}

	//--And create the image in which we will save our disparities
	Mat imgDisparity8U = Mat(imgL.rows, imgL.cols, CV_8UC1);

	calDispWithSGBM(imgL, imgR, imgDisparity8U);
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
