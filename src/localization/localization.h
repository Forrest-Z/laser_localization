#ifndef LOCALIZATION_LOCALIZATION_H
#define LOCALIZATION_LOCALIZATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#include <mutex>
#include "UKF.h"
#include "types.h"
#include "ct_icp/ct_icp.h"

namespace laser_localization
{

    class localization_options {
    public:
        std::string global_map_path;
        double filter_k;
        int frame_gap;
        int frame_size;
        int num_threads;                    // Number of threads to be used
        double downsampling_resolution;     // Downsampling resolution
        double max_correspondence_distance;  // Maximum correspondence distance between points (e.g., triming threshold)

        // 默认构造函数
        localization_options() = default;

        // 参数化构造函数
        localization_options(const std::string& map_path, double k, int gap, int size)
                : global_map_path(map_path), filter_k(k), frame_gap(gap), frame_size(size) {}

        // 复制构造函数
        localization_options(const localization_options& other)
                : global_map_path(other.global_map_path),
                  filter_k(other.filter_k),
                  frame_gap(other.frame_gap),
                  frame_size(other.frame_size),
                  num_threads(other.num_threads),
                  downsampling_resolution(other.downsampling_resolution),
                  max_correspondence_distance(other.max_correspondence_distance) {}
        // 赋值运算符
        localization_options& operator=(const localization_options& other) {
            if (this != &other) {
                global_map_path = other.global_map_path;
                filter_k = other.filter_k;
                frame_gap = other.frame_gap;
                frame_size = other.frame_size;
                num_threads = other.num_threads;
                downsampling_resolution = other.downsampling_resolution;
                max_correspondence_distance = other.max_correspondence_distance;
            }
            return *this;
        }
    };

    class localization
    {
    public:
        explicit localization(const localization_options& options);
        void predict(const Eigen::Matrix4f& odom);
        bool correct(std::vector<Eigen::Vector3d>& array_map);

        const std::vector<Eigen::Vector3d>& global_map(){return map_;}
        const std::vector<Eigen::Vector3d>& global_map_frame(){return map_frame_;}
        bool is_ready_correct(){if (ready_correct_) {ready_correct_ = false; return true;} else return false;}
        Eigen::Matrix4d get_estimate() {return estimate_.get_pos().cast<double>();}
    private:
        // option
        localization_options options_;

        // filter
        filter estimate_;
        bool ready_correct_ = false;
        Eigen::Matrix4f odom_base_;

        // map
        std::vector<Eigen::Vector3d> map_;
        std::vector<Eigen::Vector3d> map_frame_;

        void load_map(std::string path);
        void update_map_frame(const Eigen::Matrix4f& pose);
        void check_ready_correct();
        void sub_sample(std::vector<Eigen::Vector3d> &frame, double size_voxel);
    };
}// laser_localization

#endif //LOCALIZATION_LOCALIZATION_H
