#include "TemplateMatch.h"
#include <numeric>
#define size_template 18


void TemplateMatch::ColorThresholding(cv::Mat img_copy)
{
	for (int j = 0; j < img_copy.rows; j++)
	{
		uchar*data = img_copy.ptr<uchar>(j);
		for (int i = 0; i < img_copy.cols; i++)
		{

			if ((abs(data[3 * i] - CorlorsChosen[0]) + abs(data[3 * i + 1] - CorlorsChosen[1]) + abs(data[3 * i + 2] - CorlorsChosen[2])) < threshold)
			{
				data[3 * i] = data[3 * i + 1] = data[3 * i + 2] = 255;
			}
			else data[3 * i] = data[3 * i + 1] = data[3 * i + 2] = 0;
		}
	}
	std::cout << "img_copy.channles():"<<img_copy.channels() << std::endl;
}



  void TemplateMatch::Mouse_getColor(int event, int x, int y, int, void*)
  {
	  static cv::Point origin;
	  static cv::Rect selection;//静态变量用以开辟内存？
	  switch (event)
	  {
	  case CV_EVENT_LBUTTONDOWN:
		  origin = cv::Point(x, y);
		  break;
	  case CV_EVENT_LBUTTONUP:
		  selection.x = MIN(x, origin.x);
		  selection.y = MIN(y, origin.y);
		  selection.width = abs(x - origin.x);
		  selection.height = abs(y - origin.y);
		  getColors = true;
		  std::cout << "Colour area has been selected!" << std::endl;

		  int SumOfChannelOneColor = 0, SumOfChannelTwoColor = 0;
		  int SumOfChannelThreeColor = 0, SumOfChannelFourColor = 0;
		  for (int j = selection.y; j < selection.y + selection.height; j++)
		  {
			  uchar*data = image_r.ptr<uchar>(j);
			  for (int i = selection.x; i < selection.x + selection.width; i++)
			  {
				  SumOfChannelOneColor += data[i * 3];
				  SumOfChannelTwoColor += data[i * 3 + 1];
				  SumOfChannelThreeColor += data[i * 3 + 2];
				  //SumOfChannelFourColor += data[i * 3 + 3];
			  }
		  }
		  int SumOfPixels = selection.width*selection.height;
		  CorlorsChosen[0] = static_cast<int>(SumOfChannelOneColor / SumOfPixels);
		  CorlorsChosen[1] = static_cast<int>(SumOfChannelTwoColor / SumOfPixels);
		  CorlorsChosen[2] = static_cast<int>(SumOfChannelThreeColor / SumOfPixels);
		  std::cout << "blue green red: " << CorlorsChosen[0] << " " << CorlorsChosen[1] << " " << CorlorsChosen[2] << std::endl;
	  }
 }
  void TemplateMatch::get_cordinate(int picture)
  {
	  
		cv::Mat gray_image;
		gray_image = image.clone();
		ColorThresholding(gray_image);
		cv::Mat mask(3, 3, CV_8U, cv::Scalar(1));
		cv::morphologyEx(gray_image, gray_image, cv::MORPH_CLOSE, mask);

		std::vector<std::vector<cv::Point>>contours;
		cv::cvtColor(gray_image, gray_image, cv::COLOR_BGR2GRAY);
		assert(gray_image.channels() == 1);
		cv::findContours(gray_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		int cmin = 20; //contour的最小值
		int cmax = 120;
		std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
		while (itc != contours.end())
		{
			//std::cout << "size: " << itc->size() << std::endl;
			if (itc->size() < cmin || itc->size() > cmax)
			{
				itc = contours.erase(itc);//除去不符合要求的的轮廓
			}
			else itc++;
		}
		if (contours.size() == 1)
		{
			std::vector<std::vector<cv::Point>>::const_iterator it = contours.begin();

			int i = 0;
			while (it != contours.end())
			{
				cv::Moments mom = cv::moments(cv::Mat(*it++));
				start_point[picture] = cv::Point(mom.m10 / mom.m00, mom.m01 / mom.m00);	
				//cv::circle(image, start_point[picture][i], 2, cv::Scalar(0, 255, 0), 2);
				i++;
			}

		}

		else
		{
			std::cout << "Contours numbers are wrong，the number of contours is " << contours.size() << std::endl; 
			return;
		}
  }


// 类外初始化静态变量，开辟内存
std::vector<cv::Mat> TemplateMatch::Template_batch(12);
bool TemplateMatch::leftButtonDownFlag = false;
cv::Point TemplateMatch::originalPoint = cv::Point(0, 0);
cv::Point TemplateMatch::processPoint = cv::Point(0, 0);
cv::Point TemplateMatch::start_point[2] = { cv::Point(0, 0) ,cv::Point(0, 0)  };

cv::Mat TemplateMatch::image_l = cv::Mat::zeros(4, 5, CV_8UC3);//用来初始化左边鼠标的模板
cv::Mat TemplateMatch::image_r = cv::Mat::zeros(4, 5, CV_8UC3);
int TemplateMatch::index_template = 0;
cv::Mat TemplateMatch::image = cv::Mat::zeros(4, 5, CV_8UC3);
cv::Mat TemplateMatch::imageCopy = cv::Mat::zeros(4, 5, CV_8UC3);
bool TemplateMatch::getColors = false;
int TemplateMatch::CorlorsChosen[3];