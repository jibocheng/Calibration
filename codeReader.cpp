#include "codeReader.h"

cv::Mat codeReader::detector(const cv::Mat& src, const rs2_intrinsics& intrin)
{
	cv::Mat dst;
	src.copyTo(dst);
	cv::Ptr<cv::aruco::CharucoBoard> preboard = new cv::aruco::CharucoBoard(board);
	cv::Ptr<cv::aruco::DetectorParameters>params = cv::makePtr<cv::aruco::DetectorParameters>();
	int error;

	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << intrin.fx, 0, intrin.ppx, 0, intrin.fy, intrin.ppy, 0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << intrin.coeffs[0], intrin.coeffs[1], intrin.coeffs[2], intrin.coeffs[3], intrin.coeffs[4]);

	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>>markerCorners;
	cv::aruco::detectMarkers(dst, cv::makePtr<cv::aruco::Dictionary>(preboard->getDictionary()), markerCorners, markerIds, params);
	std::vector<cv::Point2f> charucoCorners;
	std::vector<int> charucoIds;
	if (markerIds.size() > 0)
	{
		cv::aruco::drawDetectedMarkers(dst, markerCorners, markerIds);

		cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, dst, preboard, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
	}
	if (charucoIds.size() > 0) {
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::aruco::drawDetectedCornersCharuco(dst, charucoCorners, charucoIds, color);
		cv::Vec3d rvec0, pvec0;
		bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, preboard, cameraMatrix, distCoeffs, rvec0, pvec0);
		if (valid)
		{
			cv::drawFrameAxes(dst, cameraMatrix, distCoeffs, rvec0, pvec0, 0.1f);
			rvecs = rvec0;
			pvecs = pvec0;
			emit T_cam_sender(pvecs,rvecs);
			emit code_info_sender(0);
			
		}
		else
		{
			int error = 41;
			emit code_info_sender(error);
		}
		
	}
	return dst;
}



