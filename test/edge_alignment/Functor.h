//
// Created by ram on 17-12-18.
//

#ifndef CERES_LEARNING_FUNCTOR_H
#define CERES_LEARNING_FUNCTOR_H

#include "Type.h"

#include <ceres/ceres.h>

#include "sophus/se3.h"

class PhotometricCostFunctor : public ceres::SizedCostFunction<1 , 6>
{
public:
    PhotometricCostFunctor() = delete;
    PhotometricCostFunctor(Measurement const &pt,
                           const Image &img,
                           CamConfig::Ptr camera)
            :src_img(img), m_pt(pt.pt), measurement(pt.pixel), m_cam(camera),
             fx_(camera->fx), fy_(camera->fy), cx_(camera->cx), cy_(camera->cy)
    {
        // std::cout << this->num_residuals() << std::endl;
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        // std::cout << "Enter evaluation" << std::endl

        Eigen::Matrix<double, 6, 1> se3_;
        for (int i = 0; i != 6; ++i)
            se3_(i, 0) = parameters[0][i];

        // get current point in reference frame
        Sophus::SE3 SE3_ = Sophus::SE3::exp(se3_);
        Eigen::Vector3d pt_ = SE3_.rotation_matrix()*m_pt+SE3_.translation();
        double x_ = pt_[0], y_ = pt_[1], z_=pt_[2];
        double invz = 1.0/z_, invz_2 = invz*invz;

        // reprojection to pixel plane
        Eigen::Vector2d uv_ = m_cam->projectionTo2D(pt_);
        double u_ = uv_[0], v_ = uv_[1];

        // std::cout << "What are you doing " << std::endl;

        // not in image, skip
        if (!isInImage(u_, v_))
        {
            // std::cout << "not in image, residuals dumped! current pixel: " << u_ << ' ' << v_ << std::endl;

            residuals[0] = 0;
            if (jacobians != NULL && jacobians[0] != NULL)
                for (int i = 0; i != 6; ++i)
                    jacobians[0][i] = 0;
            return true;
        }


        // compute residuals
        residuals[0] = getPixel(u_, v_) - measurement;

        // compute jacobians
        if (jacobians != NULL && jacobians[0] != NULL) {

            Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;
            jacobian_uv_ksai(0, 0) = invz * fx_;
            jacobian_uv_ksai(0, 1) = 0;
            jacobian_uv_ksai(0, 2) = -x_ * invz_2 * fx_;
            jacobian_uv_ksai(0, 3) = -x_ * y_ * invz_2 * fx_;
            jacobian_uv_ksai(0, 4) = (1 + (x_ * x_ * invz_2)) * fx_;
            jacobian_uv_ksai(0, 5) = -y_ * invz * fx_;

            jacobian_uv_ksai(1, 0) = 0;
            jacobian_uv_ksai(1, 1) = invz * fy_;
            jacobian_uv_ksai(1, 2) = -y_ * invz_2 * fy_;
            jacobian_uv_ksai(1, 3) = -(1 + y_ * y_ * invz_2) * fy_;
            jacobian_uv_ksai(1, 4) = x_ * y_ * invz_2 * fy_;
            jacobian_uv_ksai(1, 5) = x_ * invz * fy_;

            // std::cout << "uv_ksai jacobians computed" << std::endl;
            Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;
            jacobian_pixel_uv(0, 0) = (getPixel(u_ + 1, v_) - getPixel(u_ - 1, v_)) / 2;
            jacobian_pixel_uv(0, 1) = (getPixel(u_, v_ + 1) - getPixel(u_, v_ - 1)) / 2;

            // std::cout << "pixel jacobians computed" << std::endl;

            Eigen::Matrix<double, 1, 6> jacobian_ = jacobian_pixel_uv * jacobian_uv_ksai;
            for (int i = 0; i != 6; ++i)
                jacobians[0][i] = jacobian_(0, i);
        }
        else if (jacobians == NULL)
        {
            // std::cout << "jacobians == NULL";
        }

        return true;
    }

protected:
    const bool isInImage(double &u, double &v) const
    {
        return (u-2>0 && u+2<src_img.gray.cols &&
                v-2>0 && v-2<src_img.gray.rows);
    }

    float getPixel(double const &u, double const &v) const
    {
        uchar* data = & src_img.gray.data[ int ( v ) * src_img.gray.step + int ( u ) ];
        double xx = u - floor ( u );
        double yy = v - floor ( v );
        return float (
                ( 1-xx ) * ( 1-yy ) * data[0] +
                xx* ( 1-yy ) * data[1] +
                ( 1-xx ) *yy*data[ src_img.gray.step ] +
                xx*yy*data[src_img.gray.step+1]
        );

        // return float(src_img.m_img.at<uchar>(static_cast<int>(v), static_cast<int>(u)));
    }

private:
    Sophus::SE3 m_T;
    const Image src_img;
    const Eigen::Vector3d m_pt;
    const double measurement;
    CamConfig::Ptr m_cam;
    const double fx_, fy_, cx_, cy_;
};

#endif //CERES_LEARNING_FUNCTOR_H
