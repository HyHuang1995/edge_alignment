#include "Type.h"
#include "Functor.h"

#include <iostream>
#include <fstream>

int main (int argc, char *argv[])
{
    cv::Mat edge_img;
    Eigen::Matrix3d cam;
    double param[6] = {0};

    std::vector<Measurement> pts;

    // read param
    GlobalConfig::setConfigFile("/home/ram/WorkSpace/Learning/vision/test/edge_alignment/config.yaml");

    // construct camera model
    cam << GlobalConfig::fx,  0, GlobalConfig::cx,
            0, GlobalConfig::fy, GlobalConfig::cy,
            0,  0,  1;
    CamConfig::Ptr cam_config = CamConfig::createCamConfig(cam, GlobalConfig::depth_factor);

    // read images
    auto getImgFile = []
            (int no, bool isRGB){
        auto s = std::string(GlobalConfig::data_dir
                             + (isRGB?  GlobalConfig::color_prefix: GlobalConfig::depth_prefix)
                             + std::to_string(no)+GlobalConfig::file_format);
        return s;
    };

    Image img1(cv::imread(getImgFile(GlobalConfig::source_img_no, true), cv::IMREAD_UNCHANGED),
              cv::imread(getImgFile(GlobalConfig::source_img_no, false), cv::IMREAD_UNCHANGED));
    Image img2(cv::imread(getImgFile(GlobalConfig::target_img_no, true), cv::IMREAD_UNCHANGED),
              cv::imread(getImgFile(GlobalConfig::target_img_no, false), cv::IMREAD_UNCHANGED));

    // cv::namedWindow("show_image");

    // extract edge
    cv::Canny(img1.gray, edge_img,
              GlobalConfig::canny_threshold_low,
              GlobalConfig::canny_threshold_high,
              3, true);
    // cv::imshow("edge", edge_img); //debug

    // initialize pose
    Eigen::Isometry3d T_init = Eigen::Isometry3d::Identity();
    Sophus::SE3 T(T_init.rotation(), T_init.translation());
    Eigen::Matrix<double, 6, 1> se3 = T.log();
    std::cout << "initial pose: " << se3.transpose() << std::endl;

    // select points for align
    for (int u = 10; u != img1.gray.cols-10; ++u) {
        for (int v = 10; v != img1.gray.rows-10; ++v) {
            static uchar isedge, pixel, depth;
            // get info about edge points
            isedge = edge_img.at<uchar>(v, u);
            pixel = img1.gray.at<uchar>(v, u);
            depth = img1.depth.at<uchar>(v, u);

            if (isedge != 0 && depth != 0) {
                // construct measurements
                Eigen::Vector3d pt = cam_config->projectionTo3D(u, v, depth);
                pts.push_back(Measurement(Eigen::Vector2i(u, v), pt, pixel));
            }
        }
    }

#if 1
    ceres::Problem p;
    for (auto &pt:pts) {
        ceres::CostFunction *functor = new PhotometricCostFunctor(pt, img2, cam_config);
        p.AddResidualBlock(functor, nullptr, se3.data());
    }


    // sovle least square
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.inner_iteration_tolerance = 1e-16;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;
    options.function_tolerance = 1e-15;
    options.parameter_tolerance = 1e-15;

    ceres::Solver::Summary summary;

    ceres::Solve(options, &p, &summary);

    std::cout << std::endl << summary.BriefReport() << std::endl;


    // check output
    auto T_cw = Sophus::SE3::exp(se3);
    for (auto &item : param) {
        std::cout << item << ' ';
    }
    std::cout << std::endl << std::endl;

    std::cout << se3.transpose() << std::endl;

    std::cout << "final rotation: " << std::endl << T_cw.rotation_matrix() << std::endl << std::endl;
    std::cout << "final translation: " << std::endl << T_cw.translation().transpose() << std::endl << std::endl;


    // display image
    int rows = img1.color.rows, cols = img2.color.cols;
    cv::Mat dis_img(rows, cols, CV_8UC3);
    cv::Mat img_show ( rows*2, cols*2, CV_8UC3 );

    cv::cvtColor(edge_img, dis_img, CV_GRAY2BGR);
    dis_img.copyTo ( img_show ( cv::Rect ( 0, 0, cols, rows ) ) ); // edge

    cv::cvtColor(img2.gray, dis_img, CV_GRAY2BGR);
    for (auto &pt:pts)
        dis_img.at<cv::Vec3b>(pt.v, pt.u) = img1.color.at<cv::Vec3b>(pt.v, pt.u);
    dis_img.copyTo ( img_show ( cv::Rect ( 0, rows, cols, rows ) ) );   // raw edge in target image

    cv::cvtColor(img2.gray, dis_img, CV_GRAY2BGR);
    for (auto &p:pts) {
        Eigen::Vector3d pt = p.pt;
        pt = T_cw.rotation_matrix()*pt+T_cw.translation();
        auto uv = cam_config->projectionTo2D(pt);
        dis_img.at<cv::Vec3b>(int(uv[1]), int(uv[0])) = cv::Vec3b(0, 255, 0);
    }
    dis_img.copyTo ( img_show ( cv::Rect ( cols, rows, cols, rows ) ) );  // edge after alignment

    cv::imshow("result", img_show);

    cv::waitKey(-1);

#endif

#if 1
    Eigen::Quaterniond q1(-0.3879, 0.7907, 0.4393, -0.1770);
    Eigen::Quaterniond q2(-0.3869, 0.7929, 0.4368, -0.1755);
    Eigen::Matrix3d rot1= q1.toRotationMatrix();
    Eigen::Matrix3d rot2 = q2.toRotationMatrix();

    Eigen::Matrix3d truth1 = rot1.transpose()*rot2;
    Eigen::Matrix3d truth2 = rot1*rot2.transpose();

    std::cout << truth1 << std::endl << std::endl;
    std::cout << truth1.transpose() << std::endl << std::endl;
    std::cout << truth2 << std::endl << std::endl;
    std::cout << truth2.transpose() << std::endl << std::endl;
    // cv::imshow("show_image", gray1);
#endif
    // cv::waitKey(-1);

    return 0;
}