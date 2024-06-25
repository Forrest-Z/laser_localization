#ifndef LOCALIZATION_UKF_H
#define LOCALIZATION_UKF_H
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <ros/ros.h>
namespace laser_localization
{
    class filter
    {
    public:
//        filter(rclcpp::Node *node);
        filter(ros::NodeHandle &nh);
        Eigen::Vector3f predict(const Eigen::Matrix4f& update);
        Eigen::Vector3f update(const Eigen::Vector3f& p, const Eigen::Quaternionf& a);
        void set_pos(const Eigen::Vector3f& p, const Eigen::Quaternionf& a);
        Eigen::Matrix4f get_pos();
    private:
        float k1_;
        bool first_ = true;

        Eigen::Vector3f pos_;
        Eigen::Quaternionf angular_;
//        rclcpp::Node *node_;
        ros::NodeHandle nh_;
        float distance = 0;
    };

    class UKF
    {
    public:
        UKF();

    };
}
#endif //LOCALIZATION_UKF_H

