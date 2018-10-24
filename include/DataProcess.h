#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <range.h>

#define num_corners 9
bool writeMatToFile(cv::Mat& m, const char* filename);
bool readMatFromFile(cv::Mat& m, const char* filename);
const float rt = 1.4142; // root of 2
class DataProcess
{
private:
	const double pi = 3.1415926;
	const float l = 33.5; // length of each block on the chessboard (millimeter)

public:
	DataProcess();
	~DataProcess();

	// this vector needs to be initialized(with size being specified)
	std::vector<cv::Point3f> camera_coordinates = std::vector<cv::Point3f> (num_corners);
    std::ofstream camera_fout;
    std::ofstream world_fout;
	cv::Mat camera_Matrix = cv::Mat (4, num_corners, CV_32F);
	cv::Mat world_Matrix = cv::Mat (4, num_corners, CV_32F);
	cv::Mat transfer_Matrix =cv::Mat (4, 4, CV_32F);
	cv::Mat keypoints_camera_Matrix = cv::Mat(4, 3, CV_32F);
	cv::Mat keypoints_world_Matrix = cv::Mat(4, 3, CV_32F);
	std::vector<cv::Point2f> corners_r;
	std::vector<cv::Point2f> corners_l;
	std::vector<cv::Point2f> imagecorners;
    std::chrono::milliseconds start_time;
    std::chrono::milliseconds time;

	void getTime();
    bool calculate_T_the_whole();
	bool find_camera_coordinates(cv::Mat &chessboard, cv::Size boardSize);
	void mapTo3D();
	bool prepareChessBoardMatrices();
	void mapChessBoardTo3D();
	void test_transfer_matrix();
	void process();
	bool prepareMatrix();


	cv::Point3f keyPoints_world[3];
	cv::Point3d keyPoints3D[3];
    cv::Point2d keyPoints[2][3];
	cv::Mat image_r;
	cv::Mat image_l;
};
