#pragma once
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <QObject>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_types.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

class codeReader : public QObject{

	Q_OBJECT
public:
	codeReader(cv::aruco::CharucoBoard bd):board(bd){};
	~codeReader() {};
	cv::Mat detector(const cv::Mat& src, const rs2_intrinsics& intrin);
	
	cv::Vec3d getP() { return pvecs; }
	cv::Vec3d getR() { return rvecs; }

signals:
	void T_cam_sender(const cv::Vec3d pvecs, const cv::Vec3d rvecs);
	void code_info_sender(const int error);

private:
	bool get_intrin = 0;
	cv::aruco::CharucoBoard board;
	cv::Vec3d pvecs, rvecs;
	//rs2_intrinsics rs2Intrin;

private:
	//void get_Camera_intrnisic(const rs2_intrinsics& rs2_intrin);

};

