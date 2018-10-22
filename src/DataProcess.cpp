
#include <DataProcess.h>

#include "DataProcess.h"



DataProcess::DataProcess()
{
}


DataProcess::~DataProcess()
{
}


// map coordinates in 2D to 3D
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

void DataProcess::mapChessBoardTo3D() {
    printf("mapping");
    const double cx = 652.1485214233398;
    const double cy = 394.0842399597168;
    const double f = 683.3077785416543;
    const int T = 120;
    assert( corners_l.size() == num_corners);
    assert(corners_r.size() ==  num_corners);
    for(auto corner_id: util::lang::indices(corners_l))
    {
        printf(".");

        camera_coordinates[corner_id].x =
                (2*corners_l[corner_id].x-cx)*T/(2*(corners_l[corner_id].x-corners_r[corner_id].x));
        camera_coordinates[corner_id].y =
                -(2 * corners_l[corner_id].y - cy)*T / (2 * (corners_l[corner_id].x - corners_r[corner_id].x));
        camera_coordinates[corner_id].z =
                f*T / (2 * (corners_l[corner_id].x - corners_r[corner_id].x));
    }
    printf("\nmap ChessBoard corners succeed!\n");
}
cv::Matx44f DataProcess::calculate(cv::Mat camera_coordinate_matrix, cv::Mat word_coordinate_matrix)
{

	cv::Mat X = camera_coordinate_matrix;
	cv::Mat xs = word_coordinate_matrix.rowRange(0, 0);
	cv::Mat ys = word_coordinate_matrix.rowRange(1, 1);
	cv::Mat zs = word_coordinate_matrix.rowRange(2, 2);
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

bool DataProcess::find_camera_coordinates(cv::Mat &chessboard, cv::Size boardSize) {
    bool found = cv::findChessboardCorners(chessboard, boardSize, imagecorners);
    cv::drawChessboardCorners(chessboard, boardSize, imagecorners, found);
    return found;
}

bool DataProcess::prepareMatrices() {
    printf("Preparing matrices!\n");
    // left hand coordinate system, with x-axi points to bottom of the chessboard, origin-point at left-upper corner
    float word_coordinate_data[num_corners * 4] =
            {-rt*l/2, -rt*l/2, -rt*l/2,  0,  0,   0,   l,  l,    l,
                   0,       l,     2*l,  0,  l, 2*l,   0,  l,  2*l,
              rt*l/2,  rt*l/2,  rt*l/2,  0,  0,   0,   0,  0,    0,
                   1,       1,       1,  1,  1,   1,   1,  1,    1};
    world_Matrix = cv::Mat(4, num_corners, CV_32F, word_coordinate_data);
    // needs to convertTo CV_32F by hand
    world_Matrix.convertTo(world_Matrix, CV_32F);
    cv::Mat temp(3, num_corners, CV_32F);
    for(auto i: util::lang::indices(camera_coordinates))
    {
        cv::Mat temp_col = cv::Mat(camera_coordinates[i]).clone();
        // needs to do deep copy
        cv::Mat(camera_coordinates[i]).copyTo(temp.col(i));

    }

    assert(temp.cols == num_corners);
    cv::Mat row = cv::Mat::ones(1, temp.cols, CV_32F);
    temp.copyTo(camera_Matrix.rowRange(0, 3));
    row.copyTo(camera_Matrix.row(3));
    // verify  their shape
    // (width, height)
    assert(camera_Matrix.size() == cv::Size(num_corners, 4));
    assert(world_Matrix.size() == cv::Size(num_corners, 4));
    writeMatToFile(world_Matrix, "world_Matrix_ground_truth.ext");
    printf("prepare matrices succeed!\n");
    return true;
}

cv::Mat DataProcess::calculate_T_the_whole() {
    // see opencv doc at cv::DecompTypes
    // use temporary Mat or transpose() will  not work on world_Matrix (magic)
    cv::Mat temp;
    world_Matrix.copyTo(temp);
    //std::cout<<temp<<std::endl;
    cv::Mat camera_Matrix_T = cv::Mat(num_corners, 4, CV_32F);
    cv::Mat world_Matrix_T = cv::Mat(num_corners, 4, CV_32F);

    world_Matrix_T  = temp.t();
    camera_Matrix_T = camera_Matrix.t();

    cv::solve(camera_Matrix_T, world_Matrix_T, transfer_Matrix, cv::DECOMP_NORMAL);
    cv::transpose(transfer_Matrix, transfer_Matrix);
    printf("calculate transfer succeed!\n");

}

void DataProcess::test_transfer_matrix() {

    readMatFromFile(transfer_Matrix, "transfer_matrix.ext");


    cv::Mat transfer_Matrix_T = transfer_Matrix.t();
    // std::cout<<transfer_Matrix_T<<std::endl;
    cv::Mat camera_Matrix_T = camera_Matrix.t();
    // std::cout<<camera_Matrix_T<<std::endl;
    cv::Mat world_Matrix_T = camera_Matrix_T*transfer_Matrix_T;
    // still calling it world_Matrix_T, just because I don't want to init another Mat
    world_Matrix_T = world_Matrix_T.t();
    std::cout<<world_Matrix_T<<std::endl;
    writeMatToFile(world_Matrix_T, "world_Matrix_measure.ext");
    // measure the error
    readMatFromFile(world_Matrix, "world_Matrix_ground_truth_21.ext");
    cv::Mat diff;
    cv::absdiff(world_Matrix_T, world_Matrix, diff);
    std::cout<<"The error is shown in this matrix: "<<std::endl;
    std::cout<<diff<<std::endl;
}

void writeMatToFile(cv::Mat& m, const char* filename)
{
    // Declare what you need
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    // Write to file!
    file << "m" << m;
    //std::cout<<m<<std::endl;
}

void readMatFromFile(cv::Mat& m, const char* filename)
{
    // Declare what you need
    cv::FileStorage file(filename, cv::FileStorage::READ);
    assert(file.isOpened());
    // read file!
    file["m"]>>m;
}