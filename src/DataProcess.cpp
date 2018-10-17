
#include <DataProcess.h>

#include "DataProcess.h"



DataProcess::DataProcess()
{
}


DataProcess::~DataProcess()
{
}


// map cordinates in 2D to 3D
void DataProcess::mapTo3D()
{
	const double cx = 652.1485214233398;
	const double cy = 394.0842399597168;
	const double f = 683.3077785416543;
	const int T = 120;

	_3Dpoint.x=(2*points[0].x-cx)*T/(2*(points[0].x-points[1].x));
	_3Dpoint.y= -(2 * points[0].y - cy)*T / (2 * (points[0].x - points[1].x));
	_3Dpoint.z = f*T / (2 * (points[0].x - points[1].x));

}

cv::Matx44f DataProcess::calculate(cv::Mat camera_cordinate_matrix, cv::Mat word_cordinate_matrix)
{

	cv::Mat X = camera_cordinate_matrix;
	cv::Mat xs = word_cordinate_matrix.rowRange(0, 0);
	cv::Mat ys = word_cordinate_matrix.rowRange(1, 1);
	cv::Mat zs = word_cordinate_matrix.rowRange(2, 2);
	cv::Mat coeff_1, coeff_2, coeff_3;
    // see opencv doc at cv::DecompTypes
    cv::solve(X, xs, coeff_1, cv::DECOMP_NORMAL);
    cv::solve(X, xs, coeff_2, cv::DECOMP_NORMAL);
    cv::solve(X, xs, coeff_3, cv::DECOMP_NORMAL);
	cv::Matx44f T(0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0); // transfer matrix
	T.row(0) = coeff_1;
	T.row(1) = coeff_2;
	T.row(2) = coeff_3;
	return T;
	
}

bool DataProcess::find_camera_cordinates(cv::Mat &chessboard, cv::Size boardSize) {
    bool found = cv::findChessboardCorners(chessboard, boardSize, imagecorners);
    cv::drawChessboardCorners(chessboard, boardSize, imagecorners, found);
    return found;
}
