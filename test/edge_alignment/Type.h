//
// Created by ram on 17-12-18.
//

#ifndef CERES_LEARNING_TYPE_H
#define CERES_LEARNING_TYPE_H

#include "Config.h"

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
    typedef std::shared_ptr<Image> Ptr;
    typedef std::shared_ptr<Image const> ConstPtr;

    Image(cv::Mat const &img_, cv::Mat const &depth_)
            :color(img_), depth(depth_)
    {
        if (color.type() != CV_8UC3)
            throw std::runtime_error("please construct with color image");
        if (depth.type() != CV_16UC1)
            throw std::runtime_error("please construct with right format");
        cv::cvtColor(color, gray, CV_BGR2GRAY);

        createDistanceTransfrom();
    }

    void createDistanceTransfrom() {
        cv::Canny(gray, edge,
                  GlobalConfig::canny_threshold_low,
                  GlobalConfig::canny_threshold_high,
                  3, true);

        cv::distanceTransform(cv::Scalar::all(255)-edge,
                              distance,
                              cv::DIST_L2,
                              3);

        cv::normalize(distance, distance,
                      0.0,
                      1.0,
                      cv::NORM_MINMAX);

        distance.convertTo(distance, CV_8UC1, 255.0);
    }

    const cv::Mat color, depth;
    cv::Mat gray, distance, edge;
};

class CamConfig
{
public:
    // factory function
    typedef std::shared_ptr<CamConfig> Ptr;
    static CamConfig::Ptr createCamConfig(Eigen::Matrix3d &cam, double s) {
        return CamConfig::Ptr( new CamConfig(cam, s) );
    }

    CamConfig(Eigen::Matrix3d &cam, double s)
            :fx(cam(0, 0)), fy(cam(1, 1)), cx(cam(0, 2)), cy(cam(1, 2)), scalar(s)
    {
        std::cout << "fuck camera" << std::endl;
        std::cout << this->scalar << std::endl;
    }

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

#endif //CERES_LEARNING_TYPE_H
