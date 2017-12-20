//
// Created by ram on 17-12-18.
//

#ifndef CERES_LEARNING_TYPE_H
#define CERES_LEARNING_TYPE_H

#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>

struct Measurement
{
    Measurement(Eigen::Vector2i const &uv, Eigen::Vector3d const &pt_, const double pixel_)
            :u(uv[0]), v(uv[1]), pt(pt_), pixel(pixel_) {}

    const int u, v;
    const Eigen::Vector3d pt;
    const double pixel;
};

struct Image
{
    Image(cv::Mat const &img_, cv::Mat const &depth_)
            :color(img_), depth(depth_)
    {
        if (img_.type() != CV_8UC3)
            throw std::runtime_error("please construct with color image");
        cv::cvtColor(color, gray, CV_BGR2GRAY);
    }

    const cv::Mat color, depth;
    cv::Mat gray, distance;
};

struct CamConfig
{
    // factory function
    typedef std::shared_ptr<CamConfig> Ptr;
    static CamConfig::Ptr createCamConfig(Eigen::Matrix3d &cam, double s) {
        return CamConfig::Ptr( new CamConfig(cam, s) );
    }

    CamConfig(Eigen::Matrix3d &cam, double s)
            :fx(cam(0, 0)), fy(cam(1, 1)), cx(cam(0, 2)), cy(cam(1, 2)), scalar(s)
    {}

    // projection function
    Eigen::Vector3d projectionTo3D(double u, double v, double d) const {
        double zz = d / this->scalar;
        double xx = zz* ( u- this->cx ) /this->fx;
        double yy = zz* ( v-this->cy ) /this->fy;
        return Eigen::Vector3d( xx, yy, zz );
    }

    Eigen::Vector2d projectionTo2D(Eigen::Vector3d const &pt) const{
        double u = fx*pt[0]/pt[2] + cx;
        double v = fy*pt[1]/pt[2] + cy;
        return Eigen::Vector2d (u, v);
    }

    const Eigen::Matrix3d cam;
    const double fx, fy, cx, cy, scalar;
};

class GlobalConfig
{
private:
    static std::shared_ptr<GlobalConfig> m_configPtr;
    // static double
    cv::FileStorage m_file;
    GlobalConfig() = default;

public:
    typedef std::shared_ptr<GlobalConfig> Ptr;

    static void setConfigFile(std::string const &file_)
    {
        if ( m_configPtr == nullptr )
            m_configPtr = std::shared_ptr<GlobalConfig>(new GlobalConfig);
        m_configPtr->m_file = cv::FileStorage( file_.c_str(), cv::FileStorage::READ );
        if ( !m_configPtr->m_file.isOpened() )
        {
            std::cerr<<"parameter file " << file_ <<" does not exist."<<std::endl;

            m_configPtr->m_file.release();
            return;
        }

        data_dir = GlobalConfig::get<std::string>("data_dir");
        asso_file = GlobalConfig::get<std::string>("association_file");
        color_prefix = GlobalConfig::get<std::string>("color_prefix");
        depth_prefix = GlobalConfig::get<std::string>("depth_prefix");
        file_format = GlobalConfig::get<std::string>("file_format");

        source_img_no = GlobalConfig::get<int>("source_image_no");
        target_img_no = GlobalConfig::get<int>("target_image_no");

        canny_threshold_low = GlobalConfig::get<double>("canny_threshold_low");
        canny_threshold_high = GlobalConfig::get<double>("canny_threshold_high");

        fx = GlobalConfig::get<double>("fx");
        fy = GlobalConfig::get<double>("fy");
        cx = GlobalConfig::get<double>("cx");
        cy = GlobalConfig::get<double>("cy");

        depth_factor = GlobalConfig::get<double>("depth_factor");
    }

    // access the parameter values
    template< typename T >
    static T get( const std::string& key )
    {
        return T( GlobalConfig::m_configPtr->m_file[key] );
    }

    ~GlobalConfig(){
        if (m_file.isOpened())
            m_file.release();
    }

public:
    static std::string data_dir, color_prefix, depth_prefix, file_format, asso_file;
    static int source_img_no, target_img_no;
    static double canny_threshold_low, canny_threshold_high, fx, fy, cx, cy, depth_factor;

};

std::shared_ptr<GlobalConfig> GlobalConfig::m_configPtr = nullptr;
std::string GlobalConfig::data_dir, GlobalConfig::color_prefix,
        GlobalConfig::depth_prefix, GlobalConfig::file_format, GlobalConfig::asso_file;
int GlobalConfig::source_img_no, GlobalConfig::target_img_no;
double GlobalConfig::canny_threshold_low, GlobalConfig::canny_threshold_high,
        GlobalConfig::fx, GlobalConfig::fy, GlobalConfig::cx, GlobalConfig::cy, GlobalConfig::depth_factor;

#endif //CERES_LEARNING_TYPE_H
