#include "localization.h"
#include <cmath>
#include <iostream>
#include <utility>
#include "plyFile.h"

namespace laser_localization
{
    localization::localization(const localization_options& options):options_(options),
            estimate_((float)options.filter_k)
    {
        // load map
        load_map(options_.global_map_path);

//        Zero.rotation.w= 1;
//        Zero.rotation.x= 0;
//        Zero.rotation.y= 0;
//        Zero.rotation.z= 0;
//        Zero.translation.x = 0;
//        Zero.translation.y = 0;
//        Zero.translation.z = 0;
//        pos_ = Zero;
//
//        // init
//        map_points_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
//        std::string global_map;
//        nh_.getParam("global_map", global_map);
//        //输出global_map
//        std::cout << "-------global_map: " << global_map << std::endl;
////        pcl::io::loadPCDFile<pcl::PointXYZI>(global_map, *map_points_);
//        pcl::io::loadPCDFile<pcl::PointXYZI>("/home/hl/project/humanoid_ctr/src/hdl_graph_slam-master/map.pcd", *map_points_);
//
//        float global_resolution, global_view_resolution;
//        nh_.getParam("global_resolution", global_resolution);
//        nh_.getParam("global_view_resolution", global_view_resolution);
//
//        map_points_ = downSample(map_points_, global_resolution);
//        map_points_view_ = downSample(map_points_, global_view_resolution);
//
//        ndt_ = create_registration();
//        ndt_->setInputTarget(map_points_);
//
//        aligned_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
//        local_map_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    }

    void localization::predict(const Eigen::Matrix4f& update) {

    }

    bool localization::correct(std::shared_ptr<VoxelHashMap> local_map) {

    }

    void localization::load_map(std::string path) {
        //read ply frame file
        plyFile plyFileIn(std::move(path),
                          fileOpenMode_IN);
        char *dataIn = nullptr;
        int sizeOfPointsIn = 0;
        int numPointsIn = 0;
        plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);
        map_.reserve(numPointsIn);
        for (int i(0); i < numPointsIn; i++) {
            unsigned long long int offset =
                    (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
            Point3D new_point;
            new_point.raw_pt[0] = (*((float *) (dataIn + offset))) * 0.01;
            offset += sizeof(float);
            new_point.raw_pt[1] = (*((float *) (dataIn + offset))) * 0.01;
            offset += sizeof(float);
            new_point.raw_pt[2] = (*((float *) (dataIn + offset))) * 0.01;
            new_point.pt = new_point.raw_pt;
            new_point.alpha_timestamp = 0;
            map_.push_back(new_point);
        }
        map_.shrink_to_fit();
    }

    void localization::convert_map() {

        float max_x = 0, max_y = 0, min_x = 0, min_y = 0;
        for (const auto& mp:map_) {
            if (mp.raw_pt[0] > max_x) {max_x = mp.raw_pt[0];}
            if (mp.raw_pt[0] < min_x) {min_x = mp.raw_pt[0];}
            if (mp.raw_pt[1] > max_y) {max_y = mp.raw_pt[1];}
            if (mp.raw_pt[1] < min_y) {min_y = mp.raw_pt[1];}
        }

        float frame_gap = 30;
        float half_frame_size = 25;

        int size_x = int((max_x - min_x) / frame_gap);
        int size_y = int((max_y - min_y) / frame_gap);

        for (int i = 0;i < size_x;i ++) {
            for (int j = 0;j < size_y; j++) {
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

            }
        }
    }

//    void localization::update_local_map(const Eigen::Matrix4f& pose)
//    {
//        static bool init = false;
//        double distance;
//        nh_.getParam("local_map_size", distance);
//        Eigen::Vector2f delta = (pose.block<2, 1>(0, 3) - local_map_pose_.block<2, 1>(0, 3));
//        if (delta.norm() < distance / 3 && init)
//            return;
//        local_map_pose_ = pose;
//        local_map_points_->points.clear();
//        local_map_points_->points.reserve(map_points_->points.size());
//        for (const auto& pt:map_points_->points){
//            if (std::abs(pt.x - pose(0, 3)) + std::abs(pt.y - pose(1, 3)) < distance){
//                local_map_points_->points.emplace_back(pt);
//            }
//        }
//        ndt_ = create_registration();
//        ndt_->setInputTarget(map_points_);
//        init = true;
//    }
//
//    // odom
//    void localization::update_pos(const geometry_msgs::Twist::ConstPtr& cmd, float dt)
//    {
//        static float yaw = 0;
//        pos_.translation.x = (pos_.translation.x + cmd->linear.x * cos(yaw) * dt - cmd->linear.y * sin(yaw) * dt);
//        pos_.translation.y = (pos_.translation.y + cmd->linear.x * sin(yaw) * dt + cmd->linear.y * cos(yaw) * dt);
//        yaw += (float)cmd->angular.z * dt;
//        Eigen::Quaternionf q(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
//        pos_.rotation.x = q.x();
//        pos_.rotation.y = q.y();
//        pos_.rotation.z = q.z();
//        pos_.rotation.w = q.w();
//    }
//
//    void localization::predict(const Eigen::Matrix4f& update)
//    {
//        estimate_.predict(update);
//    }
//
//    bool localization::correct(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int times)
//    {
//        std::cout<<"start correcting !!!"<<std::endl;
//        //
//        double resolution;
//        nh_.getParam("global_frame_resolution", resolution);
//        cloud = downSample(cloud, resolution);
//        if (cloud->points.size() < 300)
//            return true;
//        // 1. predict laser pos
//        Eigen::Matrix4f predict_pos = estimate_.get_pos();
//        // 2. laser match
//        Eigen::Matrix4f laser_pos = ndt_match(std::move(cloud), predict_pos, times);
//        // 3. base_link pos
//        Eigen::Matrix4f base_pos = laser_pos;
//        Eigen::Quaternionf q(base_pos.block<3,3>(0,0));
//        // 4. update filter, get estimate pos
//        if (ndt_->getFitnessScore() < 5){
//            estimate_.update(base_pos.block<3, 1>(0, 3), q);
//            // 5. calculate map -->odom pos
//            auto filter_pos = estimate_.get_pos();
//            Eigen::Matrix4f map_odom = (filter_pos * inv_odom_base_);
//            Eigen::Quaternionf q1(map_odom.block<3,3>(0,0));
//            pos_.translation.x = map_odom(0, 3);
//            pos_.translation.y = map_odom(1, 3);
//            pos_.translation.z = map_odom(2, 3);
//            pos_.rotation.x = q1.x();
//            pos_.rotation.y = q1.y();
//            pos_.rotation.z = q1.z();
//            pos_.rotation.w = q1.w();
//            std::cout<<"end correcting !!!"<<std::endl;
//            return true;
//        }else {
//            std::cout<<" match error "<<ndt_->getFitnessScore()<<std::endl;
//            return false;
//        }
//    }
//
//    /**
//     * @brief
//     * @param laser_base
//     */
//    void localization::init_localization(const Eigen::Matrix4f& laser_base)
//    {
//        Eigen::Matrix3f R = laser_base.block<3, 3>(0, 0);
//        Eigen::Quaternionf q(R);
//        estimate_.set_pos(laser_base.block<3, 1>(0 ,3), q);
//
//        auto filter_pos = estimate_.get_pos();
//
//        Eigen::Matrix4f map_odom = (filter_pos * inv_odom_base_);
//        Eigen::Quaternionf q1(map_odom.block<3,3>(0,0));
//        pos_.translation.x = map_odom(0, 3);
//        pos_.translation.y = map_odom(1, 3);
//        pos_.translation.z = map_odom(2, 3);
//        pos_.rotation.x = q1.x();
//        pos_.rotation.y = q1.y();
//        pos_.rotation.z = q1.z();
//        pos_.rotation.w = q1.w();
//    }
//
//
//    Eigen::Matrix4f localization::ndt_match(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, Eigen::Matrix4f gauss, int times)
//    {
//        ndt_->setMaximumIterations(times);
//        ndt_->setInputSource(cloud);
//        ndt_->align(*aligned_, gauss);
//        if (ndt_->getFitnessScore() > 3){
//            std::cout<<" aligned score "<<ndt_->getFitnessScore()<<" re matching "<<std::endl;
//            ndt_->setMaximumIterations(times*2);
//            ndt_->align(*aligned_, gauss);
//        }
////        pcl::visualization::PCLVisualizer viewer("pcd viewer");
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c1(aligned_, 0, 255, 0);
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c2(map_points_, 255, 0, 0);
////        viewer.addPointCloud(map_points_, c2, "map");
////        viewer.addPointCloud(aligned_, c1, "aligned");
////        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned");
////        viewer.spin();
//        return ndt_->getFinalTransformation();
//    }
//
//    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr localization::create_registration()
//    {
//        float resolution, step_size, epsilon;
//        nh_.getParam("localization/ndt_resolution", resolution);
//        nh_.getParam("localization/ndt_step_size", step_size);
//        nh_.getParam("localization/ndt_epsilon", epsilon);
//
//        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
//        ndt->setResolution(resolution);
//        ndt->setStepSize(step_size);
//        ndt->setTransformationEpsilon(epsilon);
//        return ndt;
//    }
}

