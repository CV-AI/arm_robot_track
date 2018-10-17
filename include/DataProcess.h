#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
class DataProcess
{

	cv::Point2d thigh, shank, foot;
	const double pi = 3.1415926;

public:
	DataProcess();
	~DataProcess();
	cv::Matx44f calculate(cv::Mat, cv::Mat);
	std::vector<cv::Point3f> word_cordinates;
	std::vector<cv::Point3f> camera_cordinates;
	std::vector<cv::Point2f> imagecorners;
	bool find_camera_cordinates(cv::Mat &chessboard, cv::Size boardSize);
	cv::Mat A;
	void mapTo3D();
	cv::Point points[2];
	cv::Point3d _3Dpoint;
	cv::Mat image;
	cv::Mat image_r;
	cv::Mat image_l;
	double time = 0;
	double hip, knee, ankle;
};

