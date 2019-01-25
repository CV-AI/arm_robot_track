// ZED includes
#include <sl_zed/Camera.hpp>
#include <cmath>
#include <iostream>
#include "DataProcess.h"
#include <vector>
#include <string>
#include "KeypointTrack.h"
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);



int main() 
{
	Camera zed;
	cv::Size boardSize(3, 3); // size(width, height), so we need to initialize it with (cols, rows)
	DataProcess dataProcess;

	cv::namedWindow("LEFT", cv::WINDOW_NORMAL);
	cv::namedWindow("RIGHT", cv::WINDOW_NORMAL);

	// whether do tracking
	bool tracking_state = true;
    bool finding_transfer_matrix = true;
    bool found_chessboard_left = false;
    char mode;
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720;
	init_params.camera_fps = 60;
    // init_params.svo_input_filename.set("/home/zack/Videos/circle.svo");

	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS) 
	{
		printf("%s\n", toString(err).c_str());
		zed.close();
		return 1; 
	}

    CalibrationParameters calibrationParameters = zed.getCameraInformation().calibration_parameters;
    CameraParameters left_camera = calibrationParameters.left_cam;
    CameraParameters right_camera = calibrationParameters.right_cam;
    std::cout<<"T: "<<calibrationParameters.T<<std::endl;
    std::cout<<"left fx="<<left_camera.fx<<" fy="<<left_camera.fy<<" cx="<<left_camera.cx<<" cy="<<left_camera.cy<<std::endl;
    std::cout<<"right fx="<<right_camera.fx<<" fy="<<right_camera.fy<<" cx="<<right_camera.cx<<" cy="<<right_camera.cy<<std::endl;
    // Prepare new image size to retrieve half-resolution images
	Resolution image_size = zed.getResolution();
	int new_width = int(image_size.width);
	int new_height = int(image_size.height);
	std::cout << new_width << " " << new_height << std::endl;
	Mat image_zed_left(new_width, new_height, MAT_TYPE_8U_C3);
	Mat image_zed_right(new_width, new_height, MAT_TYPE_8U_C3);
	std::cout << image_zed_right.getDataType() << std::endl;

    cv::Mat show_in_the_begining = cv::Mat::ones(255,255, CV_8U);
    cv::imshow("Begin", show_in_the_begining);
    mode = cv::waitKey(0);
	switch (mode)
    {
        case 'p':
            cv::setMouseCallback("RIGHT", DataProcess::onMouseRight);
            cv::setMouseCallback("LEFT", DataProcess::onMouseLeft);
            while (finding_transfer_matrix)
            {
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
                    if(DataProcess::get_chessboard_roi_left&&DataProcess::get_chessboard_roi_right)
                    {
                        for(int num_picture=0; num_picture<2;num_picture++)
                        {
                            if(num_picture==0)
                            {
                                if(dataProcess.find_camera_coordinates(num_picture, dataProcess.image_l, boardSize)) {
                                    printf("Found left view corners successfully!\n");
                                    dataProcess.corners_l = dataProcess.imagecorners;
                                    found_chessboard_left = true;
                                }
                            }
                            // ensure the left and right chessboard were found at the same time;
                            if(num_picture==1 && found_chessboard_left)
                            {
                                if(dataProcess.find_camera_coordinates(num_picture, dataProcess.image_r, boardSize)) {
                                    printf("Found right view corners successfully!\n");
                                    dataProcess.corners_r = dataProcess.imagecorners;
                                    finding_transfer_matrix = false;
                                }
                            }
                        }
                        found_chessboard_left = false;
                    }
                    while(!(DataProcess::get_chessboard_roi_left&&DataProcess::get_chessboard_roi_right))
                    {
                        cv::imshow("LEFT", dataProcess.image_l);
                        cv::imshow("RIGHT", dataProcess.image_r);
                        cv::waitKey(1);
                    }
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
            dataProcess.prepareChessBoardMatrices();
            // use prepared matrices to calculate transfer matrix
            //std::cout<<"world matrix 1"<<dataProcess.world_Matrix<<std::endl;
            dataProcess.calculate_T_the_whole();

            // write the calculate matrix to file
            assert(writeMatToFile(dataProcess.transfer_Matrix, "transfer_matrix.ext"));

            dataProcess.test_transfer_matrix();
            break; // TODO: delete break after program is complte
        case 't':
            // entering tracking mode
            KeypointTrack kpt;
            bool first_frame = true;
            readMatFromFile(dataProcess.transfer_Matrix, "transfer_matrix.ext");
            cv::setMouseCallback("RIGHT", KeypointTrack::onMouseRight);
            cv::setMouseCallback("LEFT", KeypointTrack::onMouseLeft);
            while(tracking_state)
            {
                // do something
                if (zed.grab() == SUCCESS)     //消耗3ms，取图片
                {
                    zed.retrieveImage(image_zed_right, VIEW_RIGHT, MEM_CPU, new_width, new_height);//消耗5ms
                    zed.retrieveImage(image_zed_left, VIEW_LEFT, MEM_CPU, new_width, new_height);//消耗5ms

                    kpt.image_r = slMat2cvMat(image_zed_right);
                    kpt.image_l = slMat2cvMat(image_zed_left);  //格式转换

                    // 删去最后一个不需要的通道
                    cv::cvtColor(kpt.image_l, kpt.image_l, cv::COLOR_BGRA2BGR);
                    cv::cvtColor(kpt.image_r, kpt.image_r, cv::COLOR_BGRA2BGR);
                    kpt.image_r.convertTo(kpt.image_r, CV_8U);
                    kpt.image_l.convertTo(kpt.image_l, CV_8U);
                    assert(kpt.image_r.channels() == 3 && kpt.image_l.channels() == 3);

                    if(KeypointTrack::get_rois_l && KeypointTrack::get_rois_r)
                    {
                        if (first_frame)
                        {
                            cv::imwrite("firstFrameR.png", kpt.image_r);
                            cv::imwrite("firstFrameL.png", kpt.image_l);
                            for (int i = 0; i < 2; i++)
                            {
                                if (i == 0)
                                    kpt.image = kpt.image_l;
                                else
                                    kpt.image = kpt.image_r;
                                kpt.fistFrameprocess(i);
                            }
                            cv::imwrite("right.jpg", kpt.image_r);
                            cv::imwrite("left.jpg", kpt.image_l);
                            first_frame = false;
                        }
                        else
                        {
                            for (int i = 0; i < 2; i++)
                            {
                                if (i == 0)
                                    kpt.image = kpt.image_l;
                                else
                                    kpt.image = kpt.image_r;
                                kpt.frameProcessing(i);
                            }
                        }
                        for (int i = 0; i < 3; i++)
                        {
                            for (int k = 0; k < 2; k++)
                            {
                                dataProcess.keyPoints[k][i] = kpt.keyPoints[k][i];
                            }
                        }
                        dataProcess.process();
                    }
                    while(!(KeypointTrack::get_rois_r&&KeypointTrack::get_rois_l))
                    {
                        // show static image while rois are not selected
                        cv::imshow("RIGHT", kpt.image_r);
                        cv::imshow("LEFT", kpt.image_l);
                        char key = cv::waitKey(1);
                        if (key == 'q')
                        {
                            tracking_state = false;
                        }
                    }
                    cv::imshow("RIGHT", kpt.image_r);
                    cv::imshow("LEFT", kpt.image_l);
                    char key = cv::waitKey(1);
                    if (key == 'q')
                    {
                        tracking_state = false;
                    }
                }
            }
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

