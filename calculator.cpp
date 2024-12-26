#include "calculator.h"
void calculator::calculateEIH(cv::Mat_<double>& CalPose, cv::Mat_<double>& ToolPose, const int num_points, const std::string robotinputformat)
{
	std::vector<cv::Mat> R_gripper2base;
	std::vector<cv::Mat> t_gripper2base;
	std::vector<cv::Mat> R_target2cam;
	std::vector<cv::Mat> t_target2cam;
	cv::Mat R_cam2gripper = (cv::Mat_<double>(3, 3));
	cv::Mat t_cam2gripper = (cv::Mat_<double>(3, 1));

	cv::Mat tempR, tempT;

	
	for (size_t i = 0; i < num_points; i++)
	{
		//cv::Mat tmp = attitudeVectorToMatrix(CalPose.row(i), false, "zyx");
		cv::Mat tmp = attitudeVectorToMatrix(CalPose.row(i), false, "rotVec");
		vecHc.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);

		R_target2cam.push_back(tempR);
		t_target2cam.push_back(tempT);
	}

	for (size_t i = 0; i < num_points; i++)
	{
		//cv::Mat tmp = attitudeVectorToMatrix(ToolPose.row(i), false, "zyx");
		cv::Mat tmp = attitudeVectorToMatrix(ToolPose.row(i), false, robotinputformat);
		vecHg.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);

		R_gripper2base.push_back(tempR);
		t_gripper2base.push_back(tempT);
	}

	calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);

	Hcg = R_T2RT(R_cam2gripper, t_cam2gripper);
}

void calculator::calculateETH(cv::Mat_<double>& CalPose, cv::Mat_<double>& ToolPose, const int num_points, const std::string robotinputformat)
{
	std::vector<cv::Mat> R_base2gripper;
	std::vector<cv::Mat> t_base2gripper;
	std::vector<cv::Mat> R_target2cam;
	std::vector<cv::Mat> t_target2cam;

	cv::Mat R_cam2base = (cv::Mat_<double>(3, 3));
	cv::Mat t_cam2base = (cv::Mat_<double>(3, 1));

	cv::Mat tempR, tempT;

	for (size_t i = 0; i < num_points; i++)
	{
		//cv::Mat tmp = attitudeVectorToMatrix(CalPose.row(i), false, "zyx");
		cv::Mat tmp = attitudeVectorToMatrix(CalPose.row(i), false, "rotVec");
		vecHc.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);
		R_target2cam.push_back(tempR);
		t_target2cam.push_back(tempT);
	}

	for (size_t i = 0; i < num_points; i++)
	{
		//cv::Mat tmp = attitudeVectorToMatrix(ToolPose.row(i), false, "zyx");
		cv::Mat tmp = attitudeVectorToMatrix(ToolPose.row(i), false, robotinputformat);
		vecHg.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);
		tempR = tempR.t();
		tempT = -tempR * tempT;

		R_base2gripper.push_back(tempR);
		t_base2gripper.push_back(tempT);
	}

	calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam, t_target2cam, R_cam2base, t_cam2base, cv::CALIB_HAND_EYE_TSAI);
	Hcb = R_T2RT(R_cam2base, t_cam2base);
}


cv::Mat calculator::R_T2RT(cv::Mat& R, cv::Mat& T)
{
	cv::Mat RT;
	cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
		0.0, 0.0, 0.0);
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

	cv::hconcat(R1, T1, RT);
	return RT;
}

void calculator::RT2R_T(cv::Mat& RT, cv::Mat& R, cv::Mat& T)
{
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = RT(R_rect);
	T = RT(T_rect);
}

bool calculator::isRotationMatrix(const cv::Mat& R)
{
	cv::Mat tmp33 = R({ 0,0,3,3 });
	cv::Mat shouldBeIdentity;

	shouldBeIdentity = tmp33.t() * tmp33;

	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

cv::Mat calculator::eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

	//eulerAngle /= 180 / CV_PI;
	cv::Matx13d m(eulerAngle);
	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto xs = std::sin(rx), xc = std::cos(rx);
	auto ys = std::sin(ry), yc = std::cos(ry);
	auto zs = std::sin(rz), zc = std::cos(rz);

	cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
	cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
	cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);

	cv::Mat rotMat;

	if (seq == "ZYX")		rotMat = rotX * rotY * rotZ;
	else if (seq == "YZX")	rotMat = rotX * rotZ * rotY;
	else if (seq == "ZXY")	rotMat = rotY * rotX * rotZ;
	else if (seq == "XZY")	rotMat = rotY * rotZ * rotX;
	else if (seq == "YXZ")	rotMat = rotZ * rotX * rotY;
	else if (seq == "XYZ")	rotMat = rotZ * rotY * rotX;
	else if (seq == "ZYZ")  rotMat = rotZ * rotY * rotZ;
	else {
		cv::error(cv::Error::StsAssert, "Euler angle sequence string is wrong.",
			__FUNCTION__, __FILE__, __LINE__);
	}

	if (!isRotationMatrix(rotMat)) {
		cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
			__FUNCTION__, __FILE__, __LINE__);
	}

	return rotMat;
}

cv::Mat calculator::quaternionToRotatedMatrix(const cv::Vec4d& q)
{
	double w = q[0], x = q[1], y = q[2], z = q[3];

	double x2 = x * x, y2 = y * y, z2 = z * z;
	double xy = x * y, xz = x * z, yz = y * z;
	double wx = w * x, wy = w * y, wz = w * z;

	cv::Matx33d res{
		1 - 2 * (y2 + z2),	2 * (xy - wz),		2 * (xz + wy),
		2 * (xy + wz),		1 - 2 * (x2 + z2),	2 * (yz - wx),
		2 * (xz - wy),		2 * (yz + wx),		1 - 2 * (x2 + y2),
	};
	return cv::Mat(res);
}

cv::Mat calculator::attitudeVectorToMatrix(cv::Mat m, bool useQuaternion, const std::string& seq)
{
	CV_Assert(m.total() == 6 || m.total() == 10);
	if (m.cols == 1)
		m = m.t();
	cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);

	if (useQuaternion)
	{
		cv::Vec4d quaternionVec = m({ 3, 0, 4, 1 });
		quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({ 0, 0, 3, 3 }));
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
			rotVec = m({ 3, 0, 3, 1 });
		else
			rotVec = m({ 7, 0, 3, 1 });

		if (0 == seq.compare("rotVec"))
			cv::Rodrigues(rotVec, tmp({ 0, 0, 3, 3 }));
		else
			eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({ 0, 0, 3, 3 }));
	}
	tmp({ 3, 0, 1, 3 }) = m({ 0, 0, 3, 1 }).t();

	return tmp;
}
