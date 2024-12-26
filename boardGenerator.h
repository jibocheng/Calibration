#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <string>
#include <QtCore/QObject>
#include <QtGui/QImage>
#include <QtGui/QPainter>

class boardGenerator :public QObject
{
	Q_OBJECT
public:
	boardGenerator();
	~boardGenerator() {};
	void generate(const int num);
	cv::aruco::Dictionary get_dict() { return board.getDictionary(); }
	cv::Size getSize() { return imgSize; }
	cv::aruco::CharucoBoard get_board() { return board; }
signals:
	void board_img_sender(cv::Mat boardImg);
private:
	cv::aruco::Dictionary dictionary;
	cv::aruco::CharucoBoard board;
	cv::Size imgSize;
	int m_num;
};

