#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <windows.h>
#include <cmath>
#include <numeric>
#include <fstream>
#include <math.h>
#include <vector>
#include <QtCore/QObject>
#define PI acos(-1)

class calculator: public QObject
{
	Q_OBJECT
public:
	calculator() {};
	~calculator() {};
	
	void calculateEIH(cv::Mat_<double>& CalPose, cv::Mat_<double>& ToolPose, const int num_points, const std::string robotinputformat);
	void calculateETH(cv::Mat_<double>& CalPose, cv::Mat_<double>& ToolPose, const int num_points, const std::string robotinputformat);

	cv::Mat R_T2RT(cv::Mat& R, cv::Mat& T);
	void RT2R_T(cv::Mat& RT, cv::Mat& R, cv::Mat& T);
	bool isRotationMatrix(const cv::Mat& R);
	cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq);
	cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);
	cv::Mat attitudeVectorToMatrix(cv::Mat m, bool useQuaternion, const std::string& seq);

	cv::Mat getHcg() { return Hcg; }
	cv::Mat getHcb() { return Hcb; }

private:
	cv::Mat Hcg;
	cv::Mat Hcb;
	std::vector<cv::Mat> vecHg;
	std::vector<cv::Mat> vecHc;
};

