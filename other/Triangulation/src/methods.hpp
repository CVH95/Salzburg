/*
 * Original exercise v2014: Mikkel Tang Thomsen
 * Updated exercise v2017: Thorbjorn Mosekjaer Iversen
 * Updated exercise v2018: Frederik Haarslev
 *
 *      A solution to the stereopsis exercise
 *
 * methods.hpp contains methods which are irrelevant to understand for the exercise
 *
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

double tmp_x=-1;
double tmp_y=-1;
/*
 * Callback event for detecting mouse-click on images
 */
void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        //std::std::cout << "\nLeft button of the mouse is clicked - position (" << x << ", " << y << ")\n" << std::std::endl;
        tmp_x = x;
        tmp_y = y;
    }
}

struct Camera {
    cv::Mat intrinsic;
    cv::Mat transformation;
    cv::Mat distortion;
    cv::Mat projection;
    cv::Mat translation;
    cv::Mat rotation;
    double image_width;
    double image_height;

    void printData() {
        std::cout << image_width << " " << image_height << "\n" << intrinsic << "\n"
                << distortion << "\n" << transformation << "\n" << projection
                << std::endl;
    }
};

struct StereoPair {
    Camera cam1;
    Camera cam2;
};

void loadCamFromStream(std::istream & input, Camera &cam) {
    cv::Mat intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat projection = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat transformation = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rotation = cv::Mat::zeros(3, 3, CV_64F);
    double image_width, image_height;
    input.precision(20);
    input >> image_width >> image_height;

    input >> intrinsic.at<double>(0, 0) >> intrinsic.at<double>(0, 1)
            >> intrinsic.at<double>(0, 2);
    input >> intrinsic.at<double>(1, 0) >> intrinsic.at<double>(1, 1)
            >> intrinsic.at<double>(1, 2);
    input >> intrinsic.at<double>(2, 0) >> intrinsic.at<double>(2, 1)
            >> intrinsic.at<double>(2, 2);

    input >> distortion.at<double>(0, 0) >> distortion.at<double>(1, 0)
            >> distortion.at<double>(2, 0) >> distortion.at<double>(3, 0);

    /*input >> projection.at<double>(0, 0) >> projection.at<double>(0, 1)
            >> projection.at<double>(0, 2) >> projection.at<double>(0, 3);
    input >> projection.at<double>(1, 0) >> projection.at<double>(1, 1)
            >> projection.at<double>(1, 2) >> projection.at<double>(1, 3);
    input >> projection.at<double>(2, 0) >> projection.at<double>(2, 1)
            >> projection.at<double>(2, 2) >> projection.at<double>(2, 3);*/

    input >> rotation.at<double>(0, 0) >> rotation.at<double>(0, 1)
            >> rotation.at<double>(0, 2);
    input >> rotation.at<double>(1, 0) >> rotation.at<double>(1, 1)
            >> rotation.at<double>(1, 2);
    input >> rotation.at<double>(2, 0) >> rotation.at<double>(2, 1)
            >> rotation.at<double>(2, 2);
    input >> translation.at<double>(0) >> translation.at<double>(1)
            >> translation.at<double>(2);

    cv::hconcat(rotation, translation, transformation);
    cv::Mat row = cv::Mat::zeros(1, 4, CV_64F);
    row.at<double>(0, 3) = 1;
    transformation.push_back(row);

    cv::Mat tmp = intrinsic;
    cv::Mat tmp1 = cv::Mat::zeros(3, 1, CV_64F);
    cv::hconcat(tmp, tmp1, tmp);
    projection = tmp * transformation;

    cam.distortion = distortion;
    cam.intrinsic = intrinsic;
    cam.projection = projection;
    cam.transformation = transformation;
    cam.image_height = image_height;
    cam.image_width = image_width;
    cam.translation = translation;
    cam.rotation = rotation;
}

bool readStereoCameraFile(const std::string & fileNameP,
        StereoPair &stereoPair) {
    int number_of_cameras;
    Camera cam1, cam2;
    std::ifstream ifs(fileNameP.c_str());
    if (ifs) {
        ifs >> number_of_cameras;
        if (number_of_cameras == 2) {
            loadCamFromStream(ifs, cam1);
            loadCamFromStream(ifs, cam2);
            stereoPair.cam1 = cam1;
            stereoPair.cam2 = cam2;
            return true;
        }
    }
    return false;
}

cv::Mat getMouseClick(std::string imageName, cv::Mat image){
    cv::imshow(imageName,image);
    cv::setMouseCallback(imageName,CallBackFunc,NULL);

    while(tmp_x < 0){
        cv::waitKey(100);
    }
    cv::Mat m(3, 1, CV_64F);
    m.at<double>(0, 0) = tmp_x;
    m.at<double>(1, 0) = tmp_y;
    m.at<double>(2, 0) = 1;

    tmp_x = -1;
    tmp_y = -1;
    return m;
}

int loadImagesAndCalibration(int argc, char** argv, cv::Mat &img_l, cv::Mat &img_r, StereoPair &stereoPair){
    std::string calibrationFile, leftImg, rightImg;
    if (argc != 4) {
        std::cout << "Invalid arguments" << std::endl;
        std::cout << "Program usage: ./Stereopsis calibrationFile.txt leftImage.png rightImage.png" << std::endl;
        return -1;
    } else {
        calibrationFile = argv[1];
        leftImg = argv[2];
        rightImg = argv[3];
    }

    //Try to load images and calibration file
    img_l = cv::imread(leftImg, CV_LOAD_IMAGE_COLOR);
    img_r = cv::imread(rightImg, CV_LOAD_IMAGE_COLOR);
    if (img_l.empty() || img_r.empty()) {
        std::cout << "Error loading the images" << std::endl;
        return -1;
    }

    std::ifstream ifs(calibrationFile.c_str());
    if (ifs) {
        //Load calibration file
        readStereoCameraFile(calibrationFile, stereoPair);
    } else {
        std::cout << "Error opening calibration file. Calibration file must be in old OpenCV format. Calibration file must be in old OpenCV format.." << std::endl;
        return -1;
    }

    return 0;

}
