//
// Created by ram on 17-12-21.
//

#ifndef CERES_LEARNING_SOLVER_H
#define CERES_LEARNING_SOLVER_H

#include "Type.h"
#include "Functor.h"

#include <sophus/se3.h>

class Solver
{
public:
    Solver() = delete;
    Solver(Image::ConstPtr src_img_, Image::ConstPtr tar_img_, CamConfig::Ptr cam_)
            :m_T(Eigen::Isometry3d::Identity()),
             m_SE3(Sophus::SE3(m_T.rotation(), m_T.translation())),
             m_se3(m_SE3.log()),
             m_srcImg(src_img_), m_tarImg(tar_img_), m_cam(cam_)
    { }

    const Eigen::Isometry3d& getFinalTransform() const{ return m_T; }
    const Sophus::SE3& getFinalTransformSE3() const{ return m_SE3; }

    Solver* generateMeasurement()
    {
        uchar isedge, pixel;
        uint16_t depth;

        m_measures.clear();
        std::cout << (m_srcImg->depth.type() == CV_16UC1 )<< std::endl;
        for (int u = 1; u != m_srcImg->gray.cols-1; ++u) {
            for (int v = 1; v != m_srcImg->gray.rows-1; ++v) {

                // retrieve info about edge points
                isedge = m_srcImg->edge.at<uchar>(v, u);
                pixel = m_srcImg->gray.at<uchar>(v, u);
                depth = m_srcImg->depth.at<uint16_t>(v, u);

                if (isedge != 0 && depth != 0) {
                    // construct measurements
                    Eigen::Vector3d pt = m_cam->projectionTo3D(u, v, depth);
                    m_measures.push_back(Measurement(Eigen::Vector2i(u, v), pt, 0));
                }
            }
        }

        return this;
    }

    Solver* Solve()
    {
        initOptions();

        for (auto &pt:m_measures) {
            ceres::CostFunction *functor = new PhotometricCostFunctor(pt, m_tarImg, m_cam);
            m_problem.AddResidualBlock(functor, nullptr, m_se3.data()); // new ceres::CauchyLoss(0.8)
        }

        ceres::Solve(m_options, &m_problem, &m_summary);
        m_SE3 = Sophus::SE3::exp(m_se3);

        std::cout << std::endl << m_summary.BriefReport() << std::endl;

        return this;
    }

public:
    void initOptions()
    {
        m_options.linear_solver_type = ceres::DENSE_QR;

        m_options.trust_region_strategy_type = (ceres::TrustRegionStrategyType) GlobalConfig::trust_region_type;
        m_options.max_num_iterations = GlobalConfig::max_num_iterations;

        m_options.minimizer_progress_to_stdout = GlobalConfig::minimizer_out;
    }

    Eigen::Isometry3d m_T;
    Sophus::SE3 m_SE3;
    Eigen::Matrix<double, 6, 1> m_se3;

    ceres::Problem m_problem;
    ceres::Solver::Options m_options;
    ceres::Solver::Summary m_summary;

    const Image::ConstPtr m_srcImg, m_tarImg;
    const CamConfig::Ptr m_cam;

    std::vector<Measurement> m_measures;
};

#endif //CERES_LEARNING_SOLVER_H
