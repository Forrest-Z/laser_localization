#ifndef LOCALIZATION_UKF_H
#define LOCALIZATION_UKF_H
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace laser_localization
{
    class filter
    {
    public:
        explicit filter(float k);
        Eigen::Matrix4f predict(const Eigen::Matrix4f& update);
        Eigen::Matrix4f update(const Eigen::Vector3f& p, const Eigen::Quaternionf& a);
        void set_pos(const Eigen::Vector3f& p, const Eigen::Quaternionf& a);
        Eigen::Matrix4f get_pos();
    private:
        float k1_;
        bool first_ = true;

        Eigen::Vector3f pos_;
        Eigen::Quaternionf angular_;
        float distance = 0;
    };

    class UKF
    {
    public:
        UKF();

    };
}
#endif //LOCALIZATION_UKF_H

