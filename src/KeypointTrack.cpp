
#include <KeypointTrack.h>

#include "KeypointTrack.h"
#include "range.h"
#define size_template 18

using namespace cv;
using namespace std;


KeypointTrack::KeypointTrack()
{
}


KeypointTrack::~KeypointTrack()
{
}


// 本函数是对第一帧图像的处理
void KeypointTrack::fistFrameprocess(int k)
{

	for (int i = 0; i < 3; i++)
	{
		tracker_rect[k][i] = mouse_rect[k][i];
		roi_image[k][i] = image(tracker_rect[k][i]);
	}
	//findPoint(k);
	find_chessboard_center(k);
	for (int i = 0; i < 3; i++)
	{
		// add relative coordinate
		keyPoints[k][i].x += tracker_rect[k][i].x;
		keyPoints[k][i].y += tracker_rect[k][i].y;
		cv::rectangle(image, tracker_rect[k][i], cv::Scalar(0, 255, 0), 1);
	}
	
	cv::line(image, keyPoints[k][0], keyPoints[k][1], cv::Scalar(255, 0, 0), 2);
	cv::line(image, keyPoints[k][1], keyPoints[k][2], cv::Scalar(255, 0, 0), 2);
}


// 对后续视频帧的处理
void KeypointTrack::frameProcessing(int k)
{

	//cout << endl << "The current frame is: " << num_frame << endl;
	for (int i = 0; i < 3; i++)
	{
	    cv::Point center = keyPoints[k][i];
		tracker_rect[k][i].x = center.x - tracker_rect[k][i].width/2;
		tracker_rect[k][i].y = center.y - tracker_rect[k][i].height/2;
		roi_image[k][i] = image(tracker_rect[k][i]);
	}
	//find_harriscorners(k);
	find_chessboard_center(k);
	for (int i = 0; i < 3; i++)
	{
		// add relative coordinate
		keyPoints[k][i].x += tracker_rect[k][i].x;
		keyPoints[k][i].y += tracker_rect[k][i].y;


		circle(image, keyPoints[k][i], 2, Scalar(0, 0, 0), 2);
		cv::rectangle(image, tracker_rect[k][i], cv::Scalar(0, 255, 0), 1);
		//cout << "The position is:" << tracker_rect[i] << endl;
	}
	cv::line(image, keyPoints[k][0], keyPoints[k][1], cv::Scalar(255, 0, 0), 2);
	cv::line(image, keyPoints[k][1], keyPoints[k][2], cv::Scalar(255, 0, 0), 2);
}


// 获取标记的中心点
// find the center of chessboard center;
void KeypointTrack::findPoint(int k)
{
	cv::Mat gray_image;

	for (int i = 0; i < 3; i++)
	{
		gray_image = roi_image[k][i].clone();
		std::vector<cv::Point> corners;

        cv::cvtColor(gray_image, gray_image, cv::COLOR_BGR2GRAY);
        // InputArray image, OutputArray corners, int maxCorners, double qualityLevel, double minDistance
        cv::goodFeaturesToTrack(gray_image, corners, 1, 0.01, 4);
        // set to the corner in center
		keyPoints[k][i].x = corners[0].x;
		keyPoints[k][i].y = corners[0].y;
	}
}

void KeypointTrack::find_harriscorners(int k) {

    cv::Mat gray_image;
    cv::Mat dst_norm, dst_norm_scaled;
    for (int i = 0; i < 3; i++)
    {
        gray_image = roi_image[k][i].clone();
        cv::Point sum;
        int num_corners = 0;
        cv::cvtColor(gray_image, gray_image, COLOR_BGR2GRAY);
        cv::Mat dst = cv::Mat::zeros( gray_image.size(), CV_32FC1 );
        cornerHarris( gray_image, dst, blockSize, apertureSize, k );
        normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        convertScaleAbs( dst_norm, dst_norm_scaled );
        for( int i = 0; i < dst_norm.rows ; i++ )
        {
            for( int j = 0; j < dst_norm.cols; j++ )
            {
                if( (int) dst_norm.at<float>(i,j) > harris_corner_thresh)
                {
                    sum += Point(j,i);
                    num_corners += 1;
                }
            }
        }
        sum = sum/num_corners;
        // InputArray image, OutputArray corners, int maxCorners, double qualityLevel, double minDistance
        keyPoints[k][i].x = sum.x;
        keyPoints[k][i].y = sum.y;
    }
}
void KeypointTrack::find_chessboard_center(int k) {
    cv::Mat gray_image;
    cv::Mat dst_norm, dst_norm_scaled;

    for (int i = 0; i < 3; i++)
    {
        gray_image = roi_image[k][i].clone();
        cv::cvtColor(gray_image, gray_image, COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(gray_image, chessboard_size, chessboard_corners);
        keyPoints[k][i].x = chessboard_corners[4].x;
        keyPoints[k][i].y = chessboard_corners[4].y;
    }
}
void KeypointTrack::onMouseLeft(int event, int x, int y, int flags, void *param) {
    static cv::Point cursor;
    switch (event)
    {

        case EVENT_LBUTTONDOWN:
            cursor = cv::Point(x, y);
            break;
        case EVENT_LBUTTONUP:

            mouse_rect[0][rect_id_l].x = cursor.x;
            mouse_rect[0][rect_id_l].y = cursor.y;
            mouse_rect[0][rect_id_l].width = abs(x - cursor.x);
            mouse_rect[0][rect_id_l].height = abs(y - cursor.y);
            if(rect_id_l==2)
            {
                get_rois_l = true;
            }
            rect_id_l +=1;
    }
}

void KeypointTrack::onMouseRight(int event, int x, int y, int flags, void *param) {
    //Mat img = mouse_image_r.clone();
    static cv::Point cursor;
    switch (event)
    {

        case EVENT_LBUTTONDOWN:
            cursor = cv::Point(x, y);
            break;
        case EVENT_LBUTTONUP:
            mouse_rect[1][rect_id_r].x = cursor.x;
            mouse_rect[1][rect_id_r].y = cursor.y;
            mouse_rect[1][rect_id_r].width = abs(x - cursor.x);
            mouse_rect[1][rect_id_r].height = abs(y - cursor.y);
            if(rect_id_r==2)
            {
                get_rois_r = true;
            }
            rect_id_r +=1;
    }
}

bool KeypointTrack::get_rois_l = false;
bool KeypointTrack::get_rois_r = false;
int KeypointTrack::rect_id_r = 0;
int KeypointTrack::rect_id_l = 0;
cv::Rect KeypointTrack::mouse_rect[2][3] = {{cv::Rect(0, 0, 4, 4), cv::Rect(0, 0, 4, 4), cv::Rect(0, 0, 4, 4)},
                                            {cv::Rect(0, 0, 4, 4), cv::Rect(0, 0, 4, 4), cv::Rect(0, 0, 4, 4)}};








