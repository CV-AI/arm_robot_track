
#include <DataProcess.h>
using namespace std::chrono; // something about time
bool writeMatToFile(cv::Mat& m, const char* filename)
{
    printf("trying to open file!\n");
    // Declare what you need
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    assert(file.isOpened());
    printf("writing file: file is opened!\n");
    // Write to file!
    file << "m" << m;
    //std::cout<<m<<std::endl;
    return true;
}

bool readMatFromFile(cv::Mat& m, const char* filename)
{
    // Declare what you need
    cv::FileStorage file(filename, cv::FileStorage::READ);
    assert(file.isOpened());
    // read file!
    file["m"]>>m;
    return true;
}
DataProcess::DataProcess()
{
    camera_fout.open("camera_data.txt");
    world_fout.open("world_data.txt");
    // start counting when the instance is created
    start_time = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    num_corners = 9;
}


DataProcess::~DataProcess()
{
    camera_fout.close();
}


void DataProcess::getTime() {

    // time: duration
    time = duration_cast< milliseconds >(system_clock::now().time_since_epoch()) - start_time;
    std::cout << "time:  " << time.count() << "  " << std::endl;
    // time in milliseconds
    camera_fout << time.count() << "  " ;
}
// map coordinates in 2D to 3D
void DataProcess::mapTo3D()
{
    for(int i =0;i<3; i++)
    {
        keyPoints3D[i].x=(keyPoints[0][i].x-cx)*T/(keyPoints[0][i].x-keyPoints[1][i].x);
        keyPoints3D[i].y= -(keyPoints[0][i].y - cy)*T / (keyPoints[0][i].x - keyPoints[1][i].x);
        keyPoints3D[i].z = f*T / (keyPoints[0][i].x - keyPoints[1][i].x);
    }
}

void DataProcess::mapChessBoardTo3D() {
    printf("mapping");
    assert( corners_l.size() == num_corners);
    assert(corners_r.size() ==  num_corners);
    for(auto corner_id: util::lang::indices(corners_l))
    {
        printf(".");

        camera_coordinates[corner_id].x =
                (corners_l[corner_id].x-cx)*T/(corners_l[corner_id].x-corners_r[corner_id].x);
        camera_coordinates[corner_id].y =
                -(corners_l[corner_id].y - cy)*T / (corners_l[corner_id].x - corners_r[corner_id].x);
        camera_coordinates[corner_id].z =
                f*T / (corners_l[corner_id].x - corners_r[corner_id].x);
    }
    printf("\nmap ChessBoard corners succeed!\n");
}


bool DataProcess::find_camera_coordinates(cv::Mat &chessboard, cv::Size boardSize) {
    bool found = cv::findChessboardCorners(chessboard, boardSize, imagecorners);
    cv::drawChessboardCorners(chessboard, boardSize, imagecorners, found);
    return found;
}

void DataProcess::prepareChessBoardMatrices() {
    printf("Preparing matrices!\n");
    // left hand coordinate system, with x-axi points to bottom of the chessboard, origin-point at left-upper corner
    float world_coordinate_data[] =
            {-rt*l/2, -rt*l/2, -rt*l/2,  0,  0,   0,   l,  l,    l,
                   0,       l,     2*l,  0,  l, 2*l,   0,  l,  2*l,
              rt*l/2,  rt*l/2,  rt*l/2,  0,  0,   0,   0,  0,    0,
                   1,       1,       1,  1,  1,   1,   1,  1,    1};
    world_Matrix = cv::Mat(4, num_corners, CV_32F, world_coordinate_data);
    // needs to convertTo CV_32F by hand
    world_Matrix.convertTo(world_Matrix, CV_32F);

    cv::Mat temp(3, num_corners, CV_32F);
    for(auto i: util::lang::indices(camera_coordinates))
    {
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
    std::cout<<"world matrix 0"<<world_Matrix<<std::endl;
}


void DataProcess::calculate_T_the_whole() {
    // see opencv doc at cv::DecompTypes
    // use temporary Mat or transpose() will  not work on world_Matrix (magic)
    cv::Mat temp;
    world_Matrix.copyTo(temp);
    std::cout<<"world matrix 2"<<world_Matrix<<std::endl;
    std::cout<<"temp"<<temp<<std::endl;
    cv::Mat camera_Matrix_T = cv::Mat(num_corners, 4, CV_32F);
    cv::Mat world_Matrix_T = cv::Mat(num_corners, 4, CV_32F);
    world_Matrix_T  = temp.t();
    camera_Matrix_T = camera_Matrix.t();

    cv::solve(camera_Matrix_T, world_Matrix_T, transfer_Matrix, cv::DECOMP_NORMAL);
    cv::transpose(transfer_Matrix, transfer_Matrix);
    printf("calculate transfer matrix succeed!\n");
    // NOTE: you need to return something if the function return type is not void
}

void DataProcess::test_transfer_matrix() {
    cv::Mat world_Matrix_measure, diff;
    world_Matrix_measure = transfer_Matrix*camera_Matrix;
    std::cout<<world_Matrix_measure<<std::endl;
    std::cout<<world_Matrix<<std::endl;
    cv::absdiff(world_Matrix_measure, world_Matrix, diff);
    std::cout<<"The error is shown in this matrix: "<<std::endl;
    std::cout<<diff<<std::endl;
}

void DataProcess::process() {

    // map found key points to 3D
    mapTo3D();
    // prepare Matrices and world coordinates of keyPoints
    prepareMatrix();
    getTime();

    double a = sqrt(pow(keyPoints3D[0].x- keyPoints3D[1].x, 2) + pow(keyPoints3D[0].y - keyPoints3D[1].y, 2) + pow(keyPoints3D[0].z - keyPoints3D[1].z, 2));
    double b = sqrt(pow(keyPoints3D[1].x - keyPoints3D[2].x, 2) + pow(keyPoints3D[1].y - keyPoints3D[2].y, 2) + pow(keyPoints3D[1].z - keyPoints3D[2].z, 2));
    double c = sqrt(pow(keyPoints3D[2].x - keyPoints3D[0].x, 2) + pow(keyPoints3D[2].y - keyPoints3D[0].y, 2) + pow(keyPoints3D[2].z - keyPoints3D[0].z, 2));

    double cos = (a*a + b*b - c*c) / (2 * a*b);
    double angle = acos(cos) * 180 / pi;

    // camera coordinates
    camera_fout << a << "   ";
    camera_fout << b << "   ";
    camera_fout << keyPoints3D[0].x << "   " << keyPoints3D[0].y << "   " << keyPoints3D[0].z << "   " ;
    camera_fout << keyPoints3D[1].x << "   " << keyPoints3D[1].y << "   " << keyPoints3D[1].z << "   " ;
    camera_fout << keyPoints3D[2].x << "   " << keyPoints3D[2].y << "   " << keyPoints3D[2].z << "   " ;
    camera_fout << angle << std::endl << std::endl;

    // world coordinates
    std::cout << "The length of arm is:" << a << std::endl;
    std::cout << "The length of elbow is:" << b << std::endl;
    std::cout << "Coordinate of shoulder is:" << "(" << keyPoints_world[0].x << "," << keyPoints_world[0].y << "," << keyPoints_world[0].z << ")" << std::endl;
    std::cout << "Coordinate of elbow is:" << "(" << keyPoints_world[1].x << "," << keyPoints_world[1].y << "," << keyPoints_world[1].z << ")" << std::endl;
    std::cout << "Coordinate of wrist is:" << "(" << keyPoints_world[2].x << "," << keyPoints_world[2].y << "," << keyPoints_world[2].z << ")" << std::endl;
    std::cout << "The angle is:" << angle << std::endl << std::endl;

    world_fout << a << "   ";
    world_fout << b << "   ";
    world_fout << keyPoints_world[0].x << "   " << keyPoints_world[0].y << "   " << keyPoints_world[0].z << "   " ;
    world_fout << keyPoints_world[1].x << "   " << keyPoints_world[1].y << "   " << keyPoints_world[1].z << "   " ;
    world_fout << keyPoints_world[2].x << "   " << keyPoints_world[2].y << "   " << keyPoints_world[2].z << "   " ;
    world_fout << angle << std::endl << std::endl;

}

bool DataProcess::prepareMatrix() {
    // prepare camera_matrix
    cv::Mat temp = cv::Mat(3, 3, CV_32F);
    cv::Mat last_element = cv::Mat::ones(1, 3, CV_32F);
    for(int i = 0;i<3; i++)
    {
        cv::Mat(keyPoints3D[i]).copyTo(temp.col(i));
    }
    temp.push_back(last_element);
    temp.copyTo(keypoints_camera_Matrix);
    assert(keypoints_camera_Matrix.size() == cv::Size(3, 4));
    std::cout<<keypoints_camera_Matrix<<std::endl;
    // prepare world matrix
    keypoints_world_Matrix = transfer_Matrix*keypoints_camera_Matrix;
    // move redundant row
    temp = keypoints_world_Matrix(cv::Rect(0, 0, 3, 3));
    assert(temp.size() == cv::Size(3, 3));
    std::cout<<keypoints_world_Matrix<<std::endl;
    for(int i = 0;i<3; i++)
    {
        keyPoints_world[i] = cv::Point3d(temp.col(i));
    }
    return true;
}



