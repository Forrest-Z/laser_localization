#ifndef LOCALIZATION_LOCALIZATION_H
#define LOCALIZATION_LOCALIZATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "UKF.h"
#include "types.h"

namespace laser_localization
{

    class localization_options {
    public:
        std::string global_map_path;
        double filter_k;
        int frame_gap;
        int frame_size;
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
                  frame_size(other.frame_size) {}
        // 赋值运算符
        localization_options& operator=(const localization_options& other) {
            if (this != &other) {
                global_map_path = other.global_map_path;
                filter_k = other.filter_k;
                frame_gap = other.frame_gap;
                frame_size = other.frame_size;
            }
            return *this;
        }
    };

    class localization
    {
    public:
        explicit localization(const localization_options& options);
        void predict(const Eigen::Matrix4f& odom);
        bool correct(const VoxelHashMap& local_map);

        const std::vector<Point3D>& global_map(){return map_;}
        const std::vector<std::vector<Point3D>>& global_map_frames(){return map_frames_;}
        const std::vector<Point3D>& global_map_frame(){return map_frame_;}
        bool is_ready_correct(){if (ready_correct_) {ready_correct_ = false; return true;} else return false;}

        Eigen::Matrix4d get_estimate() {return estimate_.get_pos().cast<double>();}
    private:
        // option
        localization_options options_;

        // filter
        filter estimate_;
        bool ready_correct_ = false;

        Eigen::Matrix4f odom_base_;

        std::shared_ptr<VoxelHashMap> local_map_;

        // map
        std::vector<Point3D> map_;
        std::vector<Point3D> map_frame_;
        std::vector<std::vector<Point3D>> map_frames_;
        ArrayPoses map_frames_pose_;
        void load_map(std::string path);
        void convert_map();
        void update_map_frame(const Eigen::Matrix4f& pose);
        void check_ready_correct();
    };
}// laser_localization

#endif //LOCALIZATION_LOCALIZATION_H
