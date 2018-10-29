#pragma once
#include<opencv2/tracking/tracker.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


class KeypointTrack
{
    // parameters for harris corner detection
    const int harris_corner_thresh = 200;
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
public:
	KeypointTrack();
	~KeypointTrack();
	void fistFrameprocess(int k);
	void frameProcessing(int k);
	void findPoint(int k);
	void find_harriscorners(int k);
	void find_chessboard_center(int k);
    static void onMouseLeft(int event, int x, int y, int flags, void *param);
    static void onMouseRight(int event, int x, int y, int flags, void *param);
	cv::Mat image;
	cv::Mat image_l;
	cv::Mat image_r;
    cv::Size chessboard_size = cv::Size(3, 3);
	cv::Point2d keyPoints[2][3];
    std::vector<cv::Point2f> chessboard_corners;
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

