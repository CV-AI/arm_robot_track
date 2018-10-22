// ZED includes
#include <sl_zed/Camera.hpp>
#include <cmath>
#include <iostream>
#include "DataProcess.h"
#include "TemplateMatch.h"
#include <vector>
#include <string>

using namespace sl;

cv::Mat slMat2cvMat(Mat& input);



int main() 
{

	Camera zed;
	cv::Size boardSize(3, 7); // size(width, height), so we need to initialize it with (cols, rows)
	DataProcess dataProcess;

	cv::namedWindow("LEFT");
	cv::namedWindow("RIGHT");

	// set to true when you try to find transfer_matrix
	// set to false when you try to test transfer_matrix
	bool test = 1;
	// whether do tracking
	bool tracking_state = true;
    bool whether_continue = true;
    bool found_chessboard_left = false;
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720;
	init_params.camera_fps = 60;//֡��
    //init_params.svo_input_filename.set("/home/zack/Videos/big_board.svo");

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

	while (whether_continue)
	{
		cv::Mat image1;cv::Mat image2;
		if (zed.grab() == SUCCESS)
		{
			zed.retrieveImage(image_zed_right, VIEW_RIGHT, MEM_CPU, new_width, new_height);
			zed.retrieveImage(image_zed_left, VIEW_LEFT, MEM_CPU, new_width, new_height);

			dataProcess.image_r = slMat2cvMat(image_zed_right);
			dataProcess.image_l = slMat2cvMat(image_zed_left);

			cv::cvtColor(dataProcess.image_l, dataProcess.image_l, cv::COLOR_BGRA2BGR);
			cv::cvtColor(dataProcess.image_r, dataProcess.image_r, cv::COLOR_BGRA2BGR);
			dataProcess.image_r.convertTo(dataProcess.image_r, CV_8U);
			dataProcess.image_l.convertTo(dataProcess.image_l, CV_8U);
			assert(dataProcess.image_r.channels() == 3&& dataProcess.image_l.channels()==3);
			//find corner coordinates in both left and right view

			for(int num_picture=0; num_picture<2;num_picture++)
            {
			    if(num_picture==0)
                {
			        if(dataProcess.find_camera_coordinates(dataProcess.image_l, boardSize)) {
                        printf("Found left view corners successfully!\n");
                        dataProcess.corners_l = dataProcess.imagecorners;
                        found_chessboard_left = true;
                    }
                }
                // ensure the left and right chessboard were found at the same time;
                if(num_picture==1 && found_chessboard_left)
                {
                    if(dataProcess.find_camera_coordinates(dataProcess.image_r, boardSize)) {
                        printf("Found right view corners successfully!\n");
                        dataProcess.corners_r = dataProcess.imagecorners;
                        whether_continue = false;
                    }
                }
            }

            found_chessboard_left = false;
            cv::imshow("LEFT", dataProcess.image_l);
			cv::imshow("RIGHT", dataProcess.image_r);
			cv::waitKey(1);
		}
	}
    cv::imshow("LEFT", dataProcess.image_l);
    cv::imshow("RIGHT", dataProcess.image_r);
    cv::waitKey(0);
    // get chessboard corners, and mat them to 3D coordinate
    dataProcess.mapChessBoardTo3D();
    // prepare the world_matrix and camera-matrix
    // there is no point to prepare world_matrix, but for the sake of simplicity, let's do it
    dataProcess.prepareMatrices();
    if(!test)
    {
        // use prepared matrices to calculate transfer matrix
        dataProcess.calculate_T_the_whole();
        // write the calculate matrix to file
        writeMatToFile(dataProcess.transfer_Matrix, "transfer_matrix.ext");
    }
    if(test)
    {
        dataProcess.test_transfer_matrix();
    }
    while(tracking_state)
    {
        // do something
        tracking_state = false;
    }

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

