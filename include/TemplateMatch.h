#pragma once
#include <opencv2/opencv.hpp>
#include<iostream>
#include<vector>

#include<cmath>

class TemplateMatch
{
	void ColorThresholding(cv::Mat img_copy);
public:
 

	cv::Point detectWindowPosition;
	cv::Point minPoint;
	cv::Point maxPoint;
	static cv::Point start_point[2];
	static cv::Point originalPoint;
	static cv::Point processPoint;



	cv::Mat detectWindow;
	static cv::Mat image;
	static cv::Mat imageCopy;
	cv::Mat rectImage;
	cv::Mat ImageResult;
	static cv::Mat image_l;
	static cv::Mat image_r;
	static bool leftButtonDownFlag;
	static bool getColors;
	double minValue;
	double maxValude;

	static std::vector<cv::Mat> Template_batch;

	int resultRows;
	int resultcols;
	int threshold = 130;
	static int index_template ;
	static int CorlorsChosen[3];
	static void Mouse_getColor(int event, int x, int y, int, void*);
	void get_cordinate(int picture);// use color to get the first Template
	

};
