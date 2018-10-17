// ZED includes
#include <sl_zed/Camera.hpp>
#define size_detection_window 30
// Sample includes
#include <SaveDepth.hpp>
#include <cmath>
#include <iostream>
#include "DataProcess.h"
#include "TemplateMatch.h"
#include <vector>
#include <string>
#include <string.h>
//#include "utils.hpp"
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
cv::Size boardSize(6,6);
void writeMatToFile(cv::Matx44f& m, const char* filename)
{
	std::ofstream fout(filename);
	if (!fout)
	{
		std::cout << "File Not Opened" << std::endl;
		return;
	}
	for (int i = 0; i<m.rows; i++)
	{
		for (int j = 0; j<m.cols; j++)
		{
			fout << m(i, j) << "\t";
		}
		fout << std::endl;
	}
	fout.close();
}

int main() 
{

	Camera zed;
	 
	DataProcess dataProcess;
	std::ofstream outfile;

	outfile.open("./data.txt");
	cv::namedWindow("LEFT");
	cv::namedWindow("RIGHT");

	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720;
	init_params.camera_fps = 60;//֡��
	//init_params.depth_mode = DEPTH_MODE_NONE;
	//init_params.svo_input_filename.set("F:\\zikang\\zed-recording-video\\video.svo");

	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS) 
	{
		printf("%s\n", toString(err).c_str());
		zed.close();
		return 1; 
	}
	// Prepare new image size to retrieve half-resolution images
	Resolution image_size = zed.getResolution();
	int new_width = int(image_size.width);
	int new_height = int(image_size.height);
	std::cout << new_width << " " << new_height << std::endl;
	Mat image_zed_left(new_width, new_height, MAT_TYPE_8U_C3);
	Mat image_zed_right(new_width, new_height, MAT_TYPE_8U_C3);
	std::cout << image_zed_right.getDataType() << std::endl;
    std::vector<cv::Point2f> imagecorners_r;
    std::vector<cv::Point2f> imagecorners_l;
	bool whether_continue = 1;
	while (whether_continue)
	{
		cv::Mat image1;cv::Mat image2;
		if (zed.grab() == SUCCESS)
		{
			zed.retrieveImage(image_zed_right, VIEW_RIGHT, MEM_CPU, new_width, new_height);
			zed.retrieveImage(image_zed_left, VIEW_LEFT, MEM_CPU, new_width, new_height);

			dataProcess.image_r = slMat2cvMat(image_zed_right);
			dataProcess.image_l = slMat2cvMat(image_zed_left);
			// ɾȥ���һ������Ҫ��ͨ��
			cv::cvtColor(dataProcess.image_l, dataProcess.image_l, cv::COLOR_BGRA2BGR);
			cv::cvtColor(dataProcess.image_r, dataProcess.image_r, cv::COLOR_BGRA2BGR);
			dataProcess.image_r.convertTo(dataProcess.image_r, CV_8U);
			dataProcess.image_l.convertTo(dataProcess.image_l, CV_8U);
			assert(dataProcess.image_r.channels() == 3&& dataProcess.image_l.channels()==3);
			//dataProcess.image = dataProcess.image;
			for(int num_picture=0; num_picture<2;num_picture++)
            {
			    if(num_picture==0)
                {
			        if(dataProcess.find_camera_cordinates(dataProcess.image_l, boardSize)) {
                        printf("Find left view corners success!\n");
                        imagecorners_l = dataProcess.imagecorners;
                    }
                }
                if(num_picture==1)
                {
                    if(dataProcess.find_camera_cordinates(dataProcess.image_r, boardSize)) {
                        printf("Find right view corners success!\n");
                        imagecorners_r = dataProcess.imagecorners;
                        whether_continue = false;
                    }
                }
            }
			dataProcess.mapTo3D();

		}
	}
	cv::Mat word_cordinate = cv::Mat::zeros(4, 4, CV_32F);
	std::fstream file;
	file.open("F://track/src/data.txt"); 
	for (int i = 0; i < 67; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file >> word_cordinate.at<float>(i, j);
		}
	}
	cv::Matx44f solution = dataProcess.calculate(dataProcess.A, word_cordinate);
	writeMatToFile(solution, "./solution.txt");
	zed.close();
	return 0;
}



cv::Mat slMat2cvMat(Mat& input) {
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

