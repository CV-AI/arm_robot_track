#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <range.h>
#include <algorithm>
#include <chrono>

bool writeMatToFile(cv::Mat& m, const char* filename);
bool readMatFromFile(cv::Mat& m, const char* filename);
const float rt = 1.4142; // root of 2
class DataProcess
{
private:
	const double pi = 3.1415926;
	const float l = 33.5; // length of each block on the chessboard (millimeter)
    const double cx = 1133.39;
    const double cy = 687.311;
    const double f = 1390.04;
    const double T = 119.981;


public:
	DataProcess();
	~DataProcess();
    int num_corners = 9;
	// this vector needs to be initialized(with size being specified)
	std::vector<cv::Point3f> camera_coordinates = std::vector<cv::Point3f> (num_corners);
    std::ofstream camera_fout;
    std::ofstream world_fout;
	cv::Mat camera_Matrix = cv::Mat (4, num_corners, CV_32F);
	cv::Mat world_Matrix = cv::Mat (4, num_corners, CV_32F);
	cv::Mat transfer_Matrix =cv::Mat (4, 4, CV_32F);
	// matrix used in key point track
	cv::Mat keypoints_camera_Matrix = cv::Mat(4, 3, CV_32F);
	cv::Mat keypoints_world_Matrix = cv::Mat(4, 3, CV_32F);
	std::vector<cv::Point2f> corners_r;
	std::vector<cv::Point2f> corners_l;
	std::vector<cv::Point2f> imagecorners;
    std::chrono::milliseconds start_time;
    std::chrono::milliseconds time;

	void getTime();
    void calculate_T_the_whole();
	bool find_camera_coordinates(int, cv::Mat &chessboard, cv::Size boardSize);
	void mapTo3D();
	void prepareChessBoardMatrices();
	void mapChessBoardTo3D();
	void test_transfer_matrix();
	void process();
	bool prepareMatrix();
	static void onMouseLeft(int event, int x, int y, int flags, void *param);
	static void onMouseRight(int event, int x, int y, int flags, void *param);
	static bool get_chessboard_roi_left;
	static bool get_chessboard_roi_right;
	static cv::Rect mouse_rect_left;
	static cv::Rect mouse_rect_right;

	cv::Point3f keyPoints_world[3];
	cv::Point3f keyPoints3D[3];
    cv::Point2f keyPoints[2][3];
	cv::Mat image_r;
	cv::Mat image_l;
};
