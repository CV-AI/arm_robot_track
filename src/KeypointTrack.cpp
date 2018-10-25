
#include <KeypointTrack.h>

#include "KeypointTrack.h"
#include "range.h"
#define size_template 18

using namespace cv;
using namespace std;


KeypointTrack::KeypointTrack()
{
	// 创建跟踪器
	for (int i = 0; i < 3; i++)
	{
		for (int k = 0; k < 2; k++)
		{
			Ptr<Tracker> track = TrackerKCF::create();
			tracker[k][i] = track;
		}
	}
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
		roi_image[i] = image(tracker_rect[k][i]);
		assert(tracker[k][i]->init(image, tracker_rect[k][i]));
	}
	findPoint(k);
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
		assert(tracker[k][i]->update(image, tracker_rect[k][i]));  // assertion fails when tracker cannot locate
		roi_image[i] = image(tracker_rect[k][i]);
	}
	findPoint(k);
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
void KeypointTrack::findPoint(int k)
{
	cv::Mat gray_image;

	for (int i = 0; i < 3; i++)
	{
		gray_image = roi_image[i].clone();
		std::vector<cv::Point> corners;

        cv::cvtColor(gray_image, gray_image, cv::COLOR_BGR2GRAY);
        // InputArray image, OutputArray corners, int maxCorners, double qualityLevel, double minDistance
        cv::goodFeaturesToTrack(gray_image, corners, 1, 0.01, 4);
//        cv::Point mean;
//
//        for(auto i: util::lang::indices(corners))
//        {
//            mean += corners[i];
//        }
//        mean = mean/float(corners.size());
//        double distance_pre;
//        cv::Point temp = corners[0];
//        distance_pre = sqrt(pow(corners[0].x, 2)+pow(corners[0].y, 2));
//        for(auto i: util::lang::indices(corners))
//        {
//            if(0<i)
//            {
//
//                double distance = sqrt(pow(corners[i].x, 2)+pow(corners[i].y, 2));
//                if(distance<distance_pre)
//                {
//                    temp = corners[i];
//                }
//            }
//        }
        // set to the corner in center
		keyPoints[k][i].x = corners[0].x;
		keyPoints[k][i].y = corners[0].y;

		circle(image, keyPoints[k][i], 2, Scalar(0, 0, 0), 2);
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






