#include "localization.h"
#include <cmath>
#include <iostream>
#include <utility>
#include "plyFile.h"
#include "small_gicp/registration/registration_helper.hpp"
#include "ct_odometry.h"
#include <chrono>

namespace laser_localization
{
    localization::localization(const localization_options& options):options_(options),
            estimate_((float)options.filter_k)
    {
        // load map
        load_map(options_.global_map_path);
        sub_sample(map_, 0.5);
        update_map_frame(estimate_.get_pos());
    }

    void localization::predict(const Eigen::Matrix4f& odom) {
        // 1. filter predict
//        estimate_.predict(update);
        odom_base_ = odom;
        // 2. update map frame
        static Eigen::Matrix4f last_map_frame = Eigen::Matrix4f::Identity();
        if ((odom_base_.block<2, 1>(0, 3) - last_map_frame.block<2, 1>(0, 3)).norm() > 20){
            Eigen::Matrix4f pose = estimate_.get_pos() * odom_base_;
            update_map_frame(pose);
            last_map_frame = odom_base_;
        }

        // 3. check ready correct
        check_ready_correct();
    }

    bool localization::correct(std::vector<Eigen::Vector3d>& array_map) {

        small_gicp::RegistrationSetting setting;
        setting.num_threads = options_.num_threads;
        setting.downsampling_resolution = options_.downsampling_resolution;
        setting.max_correspondence_distance = options_.max_correspondence_distance;

        Eigen::Matrix4d T = estimate_.get_pos().cast<double>();  // Initial guess of the transformation
        Eigen::Isometry3d init_T_target_source(T);

        small_gicp::RegistrationResult result = align(map_frame_, array_map, init_T_target_source, setting);

        Eigen::Matrix4d result_T = result.T_target_source.matrix();
        if (result.converged) {
            Eigen::Quaternionf q(result_T.block<3, 3>(0, 0).cast<float>());
            Eigen::Vector3f t = result_T.block<3, 1>(0, 3).cast<float>();
            estimate_.update(t, q);
        }
        return true;
    }


    void localization::load_map(std::string path) {
        //read ply frame file
        plyFile plyFileIn(std::move(path),
                          fileOpenMode_IN);
        char *dataIn = nullptr;
        int sizeOfPointsIn = 0;
        int numPointsIn = 0;
        plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);
        sizeOfPointsIn *= 4;
        numPointsIn /= 4;
        map_.reserve(numPointsIn);
        for (int i(0); i < numPointsIn; i++) {
            unsigned long long int offset =
                    (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
            Eigen::Vector3d new_point;
            new_point[0] = (*((float *) (dataIn + offset)));
            offset += sizeof(float);
            new_point[1] = (*((float *) (dataIn + offset)));
            offset += sizeof(float);
            new_point[2] = (*((float *) (dataIn + offset)));
            map_.push_back(new_point);
        }
        map_.shrink_to_fit();
    }

    void localization::update_map_frame(const Eigen::Matrix4f& pose){
        // update map frame
        map_frame_.clear();
        int distance = 50;
        for (const auto& pt:map_){
            if (std::abs(pt[0] - pose(0, 3)) < distance &&
                std::abs(pt[1] - pose(1, 3)) < distance){
                map_frame_.push_back(pt);
            }
        }
    }

    void localization::check_ready_correct() {
        static Eigen::Vector3f last_correct_pose = Eigen::Vector3f::Zero();
        static std::chrono::steady_clock::time_point last_correct_time = std::chrono::steady_clock::now();

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        auto gap_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_correct_time).count();

        Eigen::Vector3f pos = odom_base_.block<3, 1>(0, 3);
        double distance = (pos - last_correct_pose).norm();

        if (gap_time_ms > 1000 || distance > 5){
            last_correct_time = now;
            last_correct_pose = pos;
            ready_correct_ = true;
        }
    }

    void localization::sub_sample(std::vector<Eigen::Vector3d> &frame, double size_voxel) {
        std::unordered_map<Voxel, std::vector<Eigen::Vector3d>> grid;
        for (int i = 0; i < (int) frame.size(); i++) {
            auto kx = static_cast<short>(frame[i][0] / size_voxel);
            auto ky = static_cast<short>(frame[i][1] / size_voxel);
            auto kz = static_cast<short>(frame[i][2] / size_voxel);
            grid[Voxel(kx, ky, kz)].push_back(frame[i]);
        }
        frame.resize(0);
        int step = 0; //to take one random point inside each voxel (but with identical results when lunching the SLAM a second time)
        for (const auto &n: grid) {
            if (n.second.size() > 0) {
                //frame.push_back(n.second[step % (int)n.second.size()]);
                frame.push_back(n.second[0]);
                step++;
            }
        }
    }
}

