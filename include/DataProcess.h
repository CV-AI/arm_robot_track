#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <range.h>

#define num_corners 9
class DataProcess
{
private:
	const double pi = 3.1415926;
	const float l = 40; // length of each block on the chessboard (millimeter)

public:
	DataProcess();
	~DataProcess();
	cv::Matx44f calculate(cv::Mat, cv::Mat);
	cv::Mat calculate_T_the_whole();
	// this vector needs to be initialized(with size being specified)
	std::vector<cv::Point3f> camera_coordinates = std::vector<cv::Point3f> (num_corners);

	cv::Mat camera_Matrix = cv::Mat (4, num_corners, CV_32F);
	cv::Mat world_Matrix = cv::Mat (4, num_corners, CV_32F);
	cv::Mat transfer_Matrix =cv::Mat (4, 4, CV_32F);
	std::vector<cv::Point2f> corners_r;
	std::vector<cv::Point2f> corners_l;
	std::vector<cv::Point2f> imagecorners;
	bool find_camera_coordinates(cv::Mat &chessboard, cv::Size boardSize);
	void mapTo3D();
	bool prepareMatrices();
	void mapChessBoardTo3D();
	void test_transfer_matrix();
	cv::Point points[2];
	cv::Point3d _3Dpoint;

	cv::Mat image_r;
	cv::Mat image_l;
};

