#pragma once
#include<opencv2/tracking/tracker.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


class KeypointTrack
{
public:
	KeypointTrack();
	~KeypointTrack();
	void fistFrameprocess(int k);
	void frameProcessing(int k);
	void findPoint(int k);
    static void onMouseLeft(int event, int x, int y, int flags, void *param);
    static void onMouseRight(int event, int x, int y, int flags, void *param);
	cv::Mat image;
	cv::Mat image_l;
	cv::Mat image_r;

	cv::Point2d keyPoints[2][3];

	cv::Rect2d tracker_rect[2][3];
	cv::Mat roi_image[2][3];
	cv::Ptr<cv::Tracker> tracker[2][3];

	// variables used in Mouse Call back

	static cv::Rect mouse_rect[2][3];
    static bool get_rois_l;
    static bool get_rois_r;
    static int rect_id_l;
    static int rect_id_r;
};

