//
// Created by ram on 17-12-21.
//

#ifndef CERES_LEARNING_CONFIG_H
#define CERES_LEARNING_CONFIG_H

#include <memory>
#include <iostream>

#include <opencv2/core.hpp>

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

        trust_region_type = GlobalConfig::get<int>("trust_region_type");
        minimizer_out = GlobalConfig::get<int>("minimizer_progress_to_stdout") != 0;
        max_num_iterations = GlobalConfig::get<int>("max_num_iterations");

        use_distancetranseform = GlobalConfig::get<int>("use_distance_transeform") != 0;
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

    static bool minimizer_out, use_distancetranseform;
    static int trust_region_type, max_num_iterations;

};

std::shared_ptr<GlobalConfig> GlobalConfig::m_configPtr = nullptr;
std::string GlobalConfig::data_dir, GlobalConfig::color_prefix,
        GlobalConfig::depth_prefix, GlobalConfig::file_format, GlobalConfig::asso_file;

int GlobalConfig::source_img_no, GlobalConfig::target_img_no;
double GlobalConfig::canny_threshold_low, GlobalConfig::canny_threshold_high,
        GlobalConfig::fx, GlobalConfig::fy, GlobalConfig::cx, GlobalConfig::cy, GlobalConfig::depth_factor;

int GlobalConfig::trust_region_type, GlobalConfig::max_num_iterations;

bool GlobalConfig::minimizer_out, GlobalConfig::use_distancetranseform;

#endif //CERES_LEARNING_CONFIG_H
