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

#include "methods.hpp"
#include <array>

cv::Mat constructProjectionMat(Camera cam)
{
    cv::Mat KA = cam.intrinsic;
    cv::Mat H = cam.transformation;

    // Remember to add a row of zeros so the KA matrix becomes 3x4
    cv::Mat zeros = cv::Mat::zeros(3, 1, CV_64F);
    cv::hconcat(KA, zeros, KA);
    return KA * H;
}

std::array<cv::Mat, 2> splitPp(cv::Mat proj)
{
    std::array<cv::Mat, 2> Pp;
    Pp[0] = proj(cv::Range(0, 3), cv::Range(0, 3));
    Pp[1] = proj(cv::Range(0, 3), cv::Range(3, 4));
    return Pp;
}

cv::Mat computeOpticalCenter(std::array<cv::Mat, 2> Pp)
{
    // Compute in homogeneous coordiantes
    cv::Mat one = cv::Mat::ones(1, 1, CV_64F);
    cv::Mat C = -1.0 * Pp[0].inv(cv::DECOMP_SVD) * Pp[1];
    cv::vconcat(C, one, C);
    return C;
}

cv::Mat computeFundamentalMat(cv::Mat e, cv::Mat proj_r, cv::Mat proj_l)
{
    // Create symmetric skew 'cross product matrix' from the right epipole
    cv::Mat erx = cv::Mat::zeros(3, 3, CV_64F);
    erx.at<double>(0, 1) = -e.at<double>(2);
    erx.at<double>(0, 2) = e.at<double>(1);
    erx.at<double>(1, 0) = e.at<double>(2);
    erx.at<double>(1, 2) = -e.at<double>(0);
    erx.at<double>(2, 0) = -e.at<double>(1);
    erx.at<double>(2, 1) = e.at<double>(0);

    return erx * proj_r * proj_l.inv(cv::DECOMP_SVD);
}

void drawEpipolarLine(cv::Mat &img, cv::Mat line, double width)
{
    cv::Point p1, p2;
    double x = line.at<double>(0,0), y = line.at<double>(0,1), z = line.at<double>(0,2);
    p1.x = 0;
    p1.y = -z / y;
    p2.x = width;
    p2.y = -width * x / y - z / y;

    cv::line(img, p1, p2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

std::array<cv::Mat, 2> computePluckerLine(cv::Mat M1, cv::Mat M2)
{
    std::array<cv::Mat, 2> plucker;
    plucker[0] = M1.cross(M2) / cv::norm(M2);
    plucker[1] = M2 / cv::norm(M2);
    return plucker;
}

cv::Mat computePluckerIntersect(std::array<cv::Mat, 2> plucker_1, std::array<cv::Mat, 2> plucker_2)
{
    cv::Mat mu1 = plucker_1[0], mu2 = plucker_2[0], v1 = plucker_1[1], v2 = plucker_2[1];

    // Compute the point on the left line which is closeset to the right line, and vica versa
    double v1_v2xmu2 = v1.dot(v2.cross(mu2));
    double v1v2_v1_v2xmu1 = v1.dot(v2) * v1.dot(v2.cross(mu1));
    double pow_v1xv2 = pow(norm(v1.cross(v2)), 2);
    cv::Mat M1 = (v1_v2xmu2 - v1v2_v1_v2xmu1) / pow_v1xv2 * v1 + v1.cross(mu1);

    double v2_v1xmu1 = v2.dot(v1.cross(mu1));
    double v2v1_v2_v1xmu2 = v2.dot(v1) * v2.dot(v1.cross(mu2));
    double pow_v2xv1 = pow(norm(v2.cross(v1)), 2);
    cv::Mat M2 = (v2_v1xmu1 - v2v1_v2_v1xmu2) / pow_v2xv1 * v2 + v2.cross(mu2);

    return M1 + (M2 - M1) / 2;
}

int main(int argc, char** argv)
{
    // Load calibration matrix and left and right image
    cv::Mat img_l, img_r;
    StereoPair stereoPair;
    if(loadImagesAndCalibration(argc, argv, img_l, img_r, stereoPair)){
        std::cout << "Input error" << std::endl;
        return -1;
    }


    //Use my own projection matrices
    auto proj_l = constructProjectionMat(stereoPair.cam1);
    auto proj_r = constructProjectionMat(stereoPair.cam2);
    std::cout << "Camera 1 (left) projection matrix" << std::endl << proj_l << std::endl << std::endl;
    std::cout << "Camera 2 (right) projection matrix" << std::endl << proj_r << std::endl << std::endl;

    auto Pp_l = splitPp(proj_l);
    auto Pp_r = splitPp(proj_r);
    auto C_l = computeOpticalCenter(Pp_l);
    auto C_r = computeOpticalCenter(Pp_r);
    std::cout << "Optical centerLeft: " << std::endl << C_l << std::endl << std::endl;
    std::cout << "Optical centerRight" << std::endl << C_r << std::endl << std::endl;

    cv::Mat e_l = proj_l * C_r;
    cv::Mat e_r = proj_r * C_l;
    std::cout << "Left epipole: " << std::endl << e_l << std::endl << std::endl;
    std::cout << "Right epipole:" << std::endl << e_r << std::endl << std::endl;

    auto F_lr = computeFundamentalMat(e_r, proj_r, proj_l);
    std::cout << "Fundamental matrix left to right:" << std::endl << F_lr << std::endl << std::endl;

    // Detect mouseclick in left image
    auto m_l = getMouseClick("LeftImage", img_l);

    cv::Mat e_line_r = F_lr * m_l;
    std::cout << "Right epipolar line:" << std::endl << e_line_r << std::endl << std::endl;

    drawEpipolarLine(img_r, e_line_r, stereoPair.cam2.image_width);

    // Detect mouseclick in right image
    auto m_r = getMouseClick("Right image", img_r);

    // // Project points to infinity
    auto M_inf_l = Pp_l[0].inv(cv::DECOMP_SVD) * m_l;
    auto M_inf_r = Pp_r[0].inv(cv::DECOMP_SVD) * m_r;

    auto plucker_l = computePluckerLine(C_l(cv::Range(0, 3), cv::Range(0, 1)), M_inf_l);
    auto plucker_r = computePluckerLine(C_r(cv::Range(0, 3), cv::Range(0, 1)), M_inf_r);
    std::cout << "Plucker line parameters:" << std::endl;
    std::cout << "mu_l: " << plucker_l[0] << std::endl
              << "v_l: " << plucker_l[1] << std::endl << std::endl;
    std::cout << "mu_r: " << plucker_r[0] << std::endl
              << "v_r: " << plucker_r[1] << std::endl << std::endl;

    auto intersection = computePluckerIntersect(plucker_l, plucker_r);
    std::cout << "Triangulated point:" << std::endl << intersection << std::endl;
    std::cout << "-------------------" << std::endl << std::endl;

    // Compare with OpenCV triangulation
    cv::Mat pnts3D(1, 1, CV_64FC4);
    cv::Mat cam0pnts(1, 1, CV_64FC2);
    cv::Mat cam1pnts(1, 1, CV_64FC2);
    cam0pnts.at<cv::Vec2d>(0)[0] = m_l.at<double>(0, 0);
    cam0pnts.at<cv::Vec2d>(0)[1] = m_l.at<double>(1, 0);
    cam1pnts.at<cv::Vec2d>(0)[0] = m_r.at<double>(0, 0);
    cam1pnts.at<cv::Vec2d>(0)[1] = m_r.at<double>(1, 0);
    triangulatePoints(proj_l, proj_r, cam0pnts, cam1pnts, pnts3D);
    std::cout << "OpenCV triangulation" << std::endl;
    std::cout << "Image points: " << cam0pnts << "\t" << cam1pnts << std::endl << std::endl;
    std::cout << "Triangulated point (normalized): " << std::endl << pnts3D / pnts3D.at<double>(3, 0) << std::endl << std::endl;

    std::cout << "Done" << std::endl;
    cv::destroyAllWindows();
    return 0;
}
