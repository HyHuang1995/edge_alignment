#include "Type.h"
#include "Functor.h"
#include "Solver.h"

#include <fstream>
#include <queue>

int main (int argc, char *argv[])
{
    Eigen::Matrix3d cam;

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

    auto img1_ptr = std::make_shared<Image const>(img1.color, img1.depth);
    auto img2_ptr = std::make_shared<Image const>(img2.color, img2.depth);

    Solver solver(img1_ptr, img2_ptr, cam_config);

    Sophus::SE3 T_cw = solver.generateMeasurement()->Solve()->getFinalTransformSE3();
    std::cout << std::endl << std::endl;

    std::cout << "final rotation: " << std::endl << T_cw.rotation_matrix() << std::endl << std::endl;
    std::cout << "final translation: " << std::endl << T_cw.translation().transpose() << std::endl << std::endl;

    // display image
    int rows = img1.color.rows, cols = img2.color.cols;
    cv::Mat dis_img(rows, cols, CV_8UC3);
    cv::Mat img_show ( rows*2, cols*2, CV_8UC3 );

    cv::cvtColor(img1.edge, dis_img, CV_GRAY2BGR);
    dis_img.copyTo ( img_show ( cv::Rect ( 0, 0, cols, rows ) ) ); // edge

    std::cout << (img2.distance.type() == CV_32F) << std::endl;
    cv::cvtColor(img2.distance, dis_img, CV_GRAY2BGR);
    dis_img.copyTo ( img_show ( cv::Rect ( cols, 0, cols, rows ) ) ); // edge

    cv::cvtColor(img2.gray, dis_img, CV_GRAY2BGR);
    for (auto &pt:solver.m_measures)
        dis_img.at<cv::Vec3b>(pt.v, pt.u) = img1.color.at<cv::Vec3b>(pt.v, pt.u);
    dis_img.copyTo ( img_show ( cv::Rect ( 0, rows, cols, rows ) ) );   // raw edge in target image

    cv::cvtColor(img2.gray, dis_img, CV_GRAY2BGR);
    for (auto &p:solver.m_measures) {
        Eigen::Vector3d pt = p.pt;
        pt = T_cw.rotation_matrix()*pt+T_cw.translation();
        auto uv = cam_config->projectionTo2D(pt);

        if (uv[0] > 0 && uv[1] > 0 && uv[0] < dis_img.cols && uv[1] < dis_img.rows)
            dis_img.at<cv::Vec3b>(int(uv[1]), int(uv[0])) = cv::Vec3b(0, 255, 0);
    }
    dis_img.copyTo ( img_show ( cv::Rect ( cols, rows, cols, rows ) ) );  // edge after alignment

    cv::imshow("result", img_show);

    cv::waitKey(-1);


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
#endif

    return 0;
}