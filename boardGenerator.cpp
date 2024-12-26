#include "boardGenerator.h"

boardGenerator::boardGenerator():dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),board(cv::aruco::CharucoBoard(cv::Size(5, 7), 0.04f, 0.02f, dictionary)),imgSize(cv::Size(2361, 3307))
{}

void boardGenerator::generate(const int num)
{
	m_num = num;

	if (m_num == 1)
	{
		dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
		
		board = cv::aruco::CharucoBoard(cv::Size(5, 7), 0.03f, 0.022f, dictionary);
		imgSize = cv::Size(630, 891);
	}

	cv::Mat boardImg;

	if (m_num == 0)
	{
		board.generateImage(imgSize, boardImg, 20, 1);
	}
	else if (m_num == 1)
	{
		board.generateImage(imgSize, boardImg, 90, 1);
	}
	emit board_img_sender(boardImg);
}

