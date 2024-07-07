#include "localization.h"
#include <cmath>
#include <iostream>
#include <utility>
#include "plyFile.h"
#include "ct_icp/ct_icp.h"
#include <chrono>

namespace laser_localization
{
    localization::localization(const localization_options& options):options_(options),
            estimate_((float)options.filter_k)
    {
        // load map
        load_map(options_.global_map_path);
        sub_sample_frame(map_, 0.5);
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

    bool localization::correct(const VoxelHashMap& voxel_map) {

//        ICPSummary icp_summary;
//        icp_summary = CT_ICP_GN(options, voxel_map, keypoints, trajectory_, index_frame);
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
            Point3D new_point;
            new_point.raw_pt[1] = (*((float *) (dataIn + offset))) * 0.5;
            offset += sizeof(float);
            new_point.raw_pt[0] = (*((float *) (dataIn + offset))) * 0.5;
            offset += sizeof(float);
            new_point.raw_pt[2] = (*((float *) (dataIn + offset))) * 0.5;
            new_point.pt = new_point.raw_pt;
            new_point.alpha_timestamp = 0;
            map_.push_back(new_point);
        }
        map_.shrink_to_fit();
    }

    void localization::update_map_frame(const Eigen::Matrix4f& pose){
        // update map frame
        map_frame_.clear();
        int distance = 50;
        for (const auto& pt:map_){
            if (std::abs(pt.raw_pt[0] - pose(0, 3)) < distance &&
                std::abs(pt.raw_pt[1] - pose(1, 3)) < distance){
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
        }
    }

    void localization::convert_map() {

        float max_x = 0, max_y = 0, min_x = 0, min_y = 0;
        for (const auto& mp:map_) {
            if (mp.raw_pt[0] > max_x) {max_x = mp.raw_pt[0];}
            if (mp.raw_pt[0] < min_x) {min_x = mp.raw_pt[0];}
            if (mp.raw_pt[1] > max_y) {max_y = mp.raw_pt[1];}
            if (mp.raw_pt[1] < min_y) {min_y = mp.raw_pt[1];}
        }

        float frame_gap = options_.frame_gap;
        float half_frame_size = options_.frame_size / 2;

        int size_x = int((max_x - min_x) / frame_gap);
        int size_y = int((max_y - min_y) / frame_gap);

        for (int i = 0;i < size_x;i ++) {
            for (int j = 0;j < size_y; j++) {
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                float x = min_x + half_frame_size + frame_gap * (float)i;
                float y = min_y + half_frame_size + frame_gap * (float)j;
                pose.block<3, 1>(0, 3) << x, y, 0;
                map_frames_pose_.emplace_back(pose);
            }
        }
        map_frames_.resize(map_frames_pose_.size());
        for (const auto& mp:map_) {
            for (int i = 0;i < map_frames_pose_.size();i ++) {
                if (mp.raw_pt[0] > map_frames_pose_[i](0, 3) - half_frame_size &&
                    mp.raw_pt[0] < map_frames_pose_[i](0, 3) + half_frame_size &&
                    mp.raw_pt[1] > map_frames_pose_[i](1, 3) - half_frame_size &&
                    mp.raw_pt[1] < map_frames_pose_[i](1, 3) + half_frame_size) {
                    map_frames_[i].push_back(mp);
                }
            }
        }
    }
}

