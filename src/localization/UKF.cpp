#include "localization/UKF.h"
#include  <algorithm>
#include <utility>

namespace laser_localization
{
//    filter::filter(rclcpp::Node *node):
//    node_(node)
    filter::filter(ros::NodeHandle& nh):
            nh_(nh)
    {
//        nh_.getParam("filter/k1", k1_);
        nh_.getParam("filter/k1", k1_);
        pos_.x() = 0;
        pos_.y() = 0;
        pos_.z() = 0;
    }

    Eigen::Vector3f filter::predict(const Eigen::Matrix4f& update)
    {
        Eigen::Matrix4f p = Eigen::Matrix4f::Identity();
        p.block<3,1>(0,3) = pos_;
        p.block<3,3>(0,0) = angular_.toRotationMatrix();
        p = p * update;
        pos_ = p.block<3,1>(0,3);
        angular_ = Eigen::Quaternionf(p.block<3,3>(0,0));
        distance += update.block<2, 1>(0, 3).norm();
        //std::cout<<"upd: "<<update<<std::endl;
    }

    Eigen::Vector3f filter::update(const Eigen::Vector3f& p, const Eigen::Quaternionf& a)
    {
        float k =  k1_;//1- std::min(0.2, std::max(0.001, (distance - 0.1) / 0.8));
        //std::cout<<"dis: "<<distance<<" k: "<<k<<std::endl;
        distance = 0;
        pos_ = pos_*k + p * (1 - k);
        angular_ = angular_.coeffs() * k  + a.coeffs() * (1 - k);
        angular_ = angular_.normalized();
    }

    void filter::set_pos(const Eigen::Vector3f& p, const Eigen::Quaternionf& a)
    {
        pos_ = p;
        angular_ = a;
    }

    Eigen::Matrix4f filter::get_pos()
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 1>(0, 3) = pos_;
        T.block<3, 3>(0, 0) = angular_.toRotationMatrix();
        return T;
    }
}