#include "localization/localization.h"
#include "global_localization.h"
#include <chrono>
#include <functional>
#include <memory>
//#include "rclcpp/rclcpp.hpp"
//#include <geometry_msgs/msg/twist.hpp>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include "eigen3/Eigen/Core"
#include "unistd.h"
#include "thread"
#include "fstream"
#include "laser_odometry.h"
#include "localization/tools.h"
//#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/SetBool.h"
#include <boost/bind.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

class Publisher
{
    using PointT = pcl::PointXYZI;
public:
    Publisher(ros::NodeHandle& nh)
            :nh_(nh)
    {
        // params
        nh_.param<double>("bbs/max_range", max_range_, 30.0);
        nh_.param<double>("bbs/min_tx", min_tx_, -20.0);
        nh_.param<double>("bbs/max_tx", max_tx_, 20.0);
        nh_.param<double>("bbs/min_ty", min_ty_, -20.0);
        nh_.param<double>("bbs/max_ty", max_ty_, 20.0);
        nh_.param<double>("bbs/min_theta", min_theta_, -3.14);
        nh_.param<double>("bbs/max_theta", max_theta_, 3.14);
        nh_.param<double>("bbs/map_min_z", map_min_z_, -5.0);
        nh_.param<double>("bbs/map_max_z", map_max_z_, 15.0);
        nh_.param<int>("bbs/map_width", map_width_, 330);
        nh_.param<int>("bbs/map_height", map_height_, 283);
        nh_.param<double>("bbs/map_resolution", map_resolution_, 0.05);
        nh_.param<int>("bbs/max_points_pre_cell", max_points_pre_cell_, 5);
        nh_.param<int>("bbs/map_pyramid_level", map_pyramid_level_, 6);
        nh_.param<double>("bbs/scan_min_z", scan_min_z_, -1.5);
        nh_.param<double>("bbs/scan_max_z", scan_max_z_, 1.5);
        nh_.param<double>("bbs/map_filter_resolution", map_filter_resolution_, 0.05);
        nh_.param<double>("bbs/scan_filter_resolution", scan_filter_resolution_, 0.05);
        nh_.param<double>("global_map_width", global_map_width_, 330.0);
        nh_.param<double>("global_map_height", global_map_height_, 283.0);

        nh_.param<double>("ndt/resolution", ndt_resolution_, 0.005);
        nh_.param<double>("ndt/step_size", ndt_step_size_, 0.1);
        nh_.param<double>("ndt/epsilon", ndt_epsilon_, 0.001);
        nh_.param<int>("ndt/max_iterations", ndt_max_iterations_, 30);
        nh_.param<double>("ndt/frame_resolution", ndt_frame_resolution_, 0.1);
        nh_.param<double>("odom/kf_distance", odom_kf_distance_, 2.0);
        nh_.param<int>("odom/local_map_size", odom_local_map_size_, 20);
        nh_.param<double>("ndt/local_map_resolution", ndt_local_map_resolution_, 0.005);

        nh_.param<int>("localization/mode", location_mode_, 1);
        nh_.param<std::string>("global_map", global_map_, "/home/hl/project/humanoid_ctr/src/interactive_slam-master/map_corrected.pcd");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        nh_.param<std::string>("laser_frame", laser_frame_, "velodyne_base_link");
        nh_.param<std::string>("laser_topic", laser_topic_, "/velodyne_points");
        nh_.param<std::string>("pose_save", pose_save_, "/home/hl/project/laser_localization/test/pos.txt");
        nh_.param<int>("correct_count", correct_count_, 2);
        nh_.param<double>("global_resolution", global_resolution_, 0.05);
        nh_.param<double>("global_frame_resolution", global_frame_resolution_, 0.005);
        nh_.param<double>("global_view_resolution", global_view_resolution_, 2.0);
        nh_.param<double>("local_map_size", local_map_size_, 100.0);

        nh_.param<double>("localization/ndt_resolution", localization_ndt_resolution_, 0.005);
        nh_.param<double>("localization/ndt_step_size", localization_ndt_step_size_, 0.1);
        nh_.param<double>("localization/ndt_epsilon", localization_ndt_epsilon_, 0.01);
        nh_.param<std::string>("initial_pose_save", initial_pose_save_, "/home/hl/project/laser_localization/test/init_pos.txt");



        // localization mode
        nh_.getParam("localization/mode", location_mode_);
        nh_.getParam("odom_frame", odom_frame_);
        nh_.getParam("base_link_frame", base_link_frame_);
        nh_.getParam("laser_frame", laser_frame_);
        nh_.getParam("global_map", global_map_);
        nh_.getParam("correct_count", correct_count_);

        localization_ = (new laser_localization::localization(nh_));
        global_localization_ = (new global_localization::GlobalLocalizationBBS(nh_));
        odometry = new laserOdometry(nh_);

        // global localization
        pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(global_map_, *map) == -1)
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
        global_localization_->set_global_map(map);

//        using std::placeholders::_1;
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        // ros1 service call /set_init_pose std_srvs/SetBool
        initial_service_ = nh_.advertiseService("set_init_pose", &Publisher::set_initial_position_srv, this);
        // ros1 service call /relocalization std_srvs/SetBool
        relocalization_service_ = nh.advertiseService("relocalization", &Publisher::relocalization_srv, this);

        if(location_mode_ == 3){// fake localization
//            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//                    "cmd_vel", 10, std::bind(&Publisher::cmd_callback, this, _1));
            subscription_ = nh.subscribe("cmd_vel", 10, &Publisher::cmd_callback, this);
        }else if (location_mode_ == 1 || location_mode_ == 2){
            int try_cnt = 0;
            while (ros::ok()){
                try{
                    //用ros_info方式输出base_link_frame_和laser_frame_
//                    ROS_INFO("base_link_frame_ 1: %s", base_link_frame_.c_str());
//                    ROS_INFO("laser_frame_ 1: %s", laser_frame_.c_str());
                    auto pos = buffer_->lookupTransform(
                            base_link_frame_,
                            laser_frame_, ros::Time(0));
                    laser_base_ = transform2Matrix(pos);
                    break;
                }
                catch (tf2::TransformException &ex){
                    ROS_INFO("Could not transform %s", ex.what());
                }
                sleep(1);
                try_cnt ++;
                //if (try_cnt > 10) {
                //    laser_base_ = Eigen::Matrix4f::Identity();
                //    break;
                //}
            }
//            std::string topic;
//            this->get_parameter("laser_topic", topic);
//            laser_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//                    topic, rclcpp::SensorDataQoS(), std::bind(&Publisher::laser_callback, this, _1));
//            ROS_INFO("laser_topic_ ::: %s", laser_topic_.c_str());
            laser_sub_ = nh.subscribe(laser_topic_, 10, &Publisher::laser_callback, this);

            if (location_mode_ == 2){
                while (ros::ok())
                {
                    try{
//                        ROS_INFO("odom_frame_ 2: %s", odom_frame_.c_str());
//                        ROS_INFO("base_link_frame_ 2: %s", base_link_frame_.c_str());
                        auto pos = buffer_->lookupTransform(
                                odom_frame_,
                                base_link_frame_,
                                ros::Time(0));
                        last_trans_ = transform2Matrix(pos);
                        break;
                    }
                    catch (tf2::TransformException &ex){
                        ROS_INFO("Could not transform %s", ex.what());
                    }
                    sleep(1);
                    odom_ready = true;
                }
            }
        }
        map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map", 10);
        aligned_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/aligned", 10);

        // 5 hz
//        timer_ = this->create_wall_timer(100ms, [this] { timer_callback();});
//        timer_ = nh.createTimer(ros::Duration(0.1), &Publisher::timer_callback, this);
        timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&Publisher::timer_callback, this));
        // start thread
        t1 = std::make_shared<std::thread>(&Publisher::laser_thread, this);

        // 初始化可视化工具
//        pcl::PointCloud<pcl::PointXYZI>::Ptr show(new pcl::PointCloud<pcl::PointXYZI>);
        show_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        viewer_ = simpleVis(show_);
        last_pose_ = Eigen::Matrix4f::Identity();
    }
private:

    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(1.0, 1.0, 1.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> view_h(cloud, 0, 255, 0);
        viewer->addPointCloud(cloud, view_h, "show");
        viewer->addPointCloud(cloud, view_h, "map");
        return (viewer);
    }

    void laser_thread()
    {
        Eigen::Matrix4f last_pos = Eigen::Matrix4f::Identity();
//        std::string save_txt;
//        this->get_parameter("pose_save", save_txt);
        while (true){
//            if (!laser_ready)
//                continue;
//            if (location_mode_ == 2 && !odom_ready)
//                continue;
            if(!correct_frames.empty() && !manual_relocalization){
                lock_.lock();
                auto frame = correct_frames.front();
                Eigen::Matrix4f frame_odom = correct_frames_pose.front();
                Eigen::Matrix4f odom;
                if (location_mode_ == 2){odom = odom_pose.front();}
                lock_.unlock();

                if (localization_state == 0){
                    std::cout<<"start global localization "<<std::endl;
                    // global localization
                    Eigen::Matrix4f T;
                    if (global_localization_->globalLocalization(frame, initial_pose_save_, T)){
                        if (location_mode_ == 2){
                            localization_->set_inv_odom_base(odom.inverse());
                        }else {
                            localization_->set_inv_odom_base(frame_odom.inverse());
                        }
                        localization_->init_localization(T);
                        last_pos = frame_odom;
                        std::cout<<"global localization success !!!"<<std::endl;
                        localization_state = 1;
                    }else {
                        std::cout<<" re localization again "<<std::endl;
                        if (!global_localization_->globalLocalization(frame, T)){
                            localization_state = 2;
                        }else{
                            if (location_mode_ == 2){
                                localization_->set_inv_odom_base(odom.inverse());
                            }else {
                                localization_->set_inv_odom_base(frame_odom.inverse());
                            }
                            localization_->init_localization(T);
                            last_pos = frame_odom;
                            std::cout<<"global localization success !!!"<<std::endl;
                            localization_state = 1;
                        }
                    }
                    localization_->update_local_map(T);
                }
                if (localization_state == 1){

                    localization_->update_local_map(localization_->get_estimate_pos());
                    auto s = std::chrono::system_clock::now();
                    // predict
                    Eigen::Matrix4f delta = last_pos.inverse() * frame_odom;
                    localization_->predict(delta);
                    last_pos = frame_odom;
                    // set odom
                    if (location_mode_ == 2){
                        localization_->set_inv_odom_base(odom.inverse());
                    }else {
                        localization_->set_inv_odom_base(frame_odom.inverse());
                    }
                    // correct
                    if (!localization_->correct(frame, 30)){
                        localization_state = 2;
                    }
                    auto e = std::chrono::system_clock::now();
                    std::cout<<"cost "<<std::chrono::duration_cast<std::chrono::milliseconds>(e-s).count()<<" ms"<<std::endl;
                }else {
                    std::cout<<" try global localization"<<std::endl;
                    Eigen::Matrix4f predict = localization_->get_estimate_pos();
                    Eigen::Matrix4f T;
                    if (global_localization_->globalLocalization(frame, predict, T)){
                        if (location_mode_ == 2){
                            localization_->set_inv_odom_base(odom.inverse());
                        }else {
                            localization_->set_inv_odom_base(frame_odom.inverse());
                        }
                        localization_->init_localization(T);
                        last_pos = frame_odom;
                        std::cout<<"global localization success !!!"<<std::endl;
                        localization_state = 1;
                    }else {
                        std::cout<<" re localization again "<<std::endl;
                        if (!global_localization_->globalLocalization(frame, T)){
                            localization_state = 2;
                        }else{
                            if (location_mode_ == 2){
                                localization_->set_inv_odom_base(odom.inverse());
                            }else {
                                localization_->set_inv_odom_base(frame_odom.inverse());
                            }
                            localization_->init_localization(T);
                            last_pos = frame_odom;
                            std::cout<<"global localization success !!!"<<std::endl;
                            localization_state = 1;
                        }
                    }
                }
                write_position(pose_save_, localization_->get_estimate_pos());
                lock_.lock();
                correct_frames_pose.pop();
                correct_frames.pop();
                if (location_mode_ == 2){odom_pose.pop();}
                lock_.unlock();
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }


    void timer_callback()
    {
//        ROS_INFO("Timer callback called...");
        static int cnt = 0;
        if(cnt%100 == 0)
        {// publish map
            sensor_msgs::PointCloud2 map;
            pcl::toROSMsg(*localization_->get_view_map(), map);
            map.header.stamp = ros::Time::now();
            map.header.frame_id = "map";
            map_pub_.publish(map);
//            ROS_INFO("Published /map message");
        }
        if(cnt%3 == 0 && localization_->get_aligned() != nullptr)
        {
            sensor_msgs::PointCloud2 map;
            if(localization_->get_aligned().get()->points.size() == localization_->get_aligned().get()->width * localization_->get_aligned().get()->height)
            {
                pcl::toROSMsg(*localization_->get_aligned(), map);
                map.header.stamp = ros::Time::now();
                map.header.frame_id = "map";
                aligned_pub_.publish(map);
            }
        }

        cnt ++;
    }

    void write_position_2(const std::string& file,const Eigen::Matrix4f& position)
    {
        auto pos = position;
        tf::Matrix3x3 m(pos(0), pos(4), pos(8),
                        pos(1), pos(5), pos(9),
                        pos(2), pos(6), pos(10));

        double current_angle, tmp;
        m.getEulerYPR(current_angle, tmp, tmp);


        std::ofstream out;
        out.open(file.c_str());
        auto angle = position.block<3,3>(0,0).eulerAngles(0, 1, 2);
        Eigen::Quaternionf q(position.block<3,3>(0,0));
        out<<position(12)<<","<<position(13)<<","<<position(14)<<","<<current_angle;//<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w();
        out<<std::flush;
        out.close();
    }
    void publish_pos(geometry_msgs::Transform pos)
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform = pos;
        if (localization_state != 1){
            t.transform.translation.z = 4;
        }
        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    bool set_initial_position_srv(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        set_initial_position();
        res.success = true;
        res.message = "set initial position !!!";
        return true;
    }

    bool relocalization_srv(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        manual_relocalization = true;
        while (correct_frames.empty()){
            ROS_INFO("wait frames");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        lock_.lock();
        auto frame = correct_frames.front();
        Eigen::Matrix4f frame_odom = correct_frames_pose.front();
        if (location_mode_ == 2){
            frame_odom = odom_pose.front();
        }
        lock_.unlock();

//        ROS_INFO("start global localization");
        Eigen::Matrix4f T;
        if (req.data && global_localization_->globalLocalization(frame, initial_pose_save_, T)){
            localization_->set_inv_odom_base(frame_odom.inverse());
            localization_->init_localization(T);
            ROS_INFO("global localization success");
            localization_state = 1;
            res.message = "relocalization position success";
        }else {
            ROS_INFO("re localization again");
            if (!global_localization_->globalLocalization(frame, T)){
                localization_state = 2;
                res.message = "relocalization position failed";
            }else{
                localization_->set_inv_odom_base(frame_odom.inverse());
                localization_->init_localization(T);
                ROS_INFO("global localization success");
                res.message = "relocalization position success";
                localization_state = 1;
            }
        }
        localization_->update_local_map(T);
        manual_relocalization = false;
        return true;
    }

    void cmd_callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        static auto last_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float dt = (float)std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        static auto Zero = localization_->get_pos();
        localization_->update_pos(msg, dt / 1000000.f);
        last_time = now;

        // 发布定位信息
        {
            geometry_msgs::TransformStamped t;
            t.header.stamp = ros::Time::now();
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform = localization_->get_pos();
            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
        {
            geometry_msgs::TransformStamped t;
            t.header.stamp = ros::Time::now();
            t.header.frame_id = "map";
            t.child_frame_id = "odom";
            t.transform = Zero;
            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
    }

    Eigen::Matrix4f transform2Matrix(const geometry_msgs::TransformStamped& trans)
    {
        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf q;

        M(0, 3) = (float)trans.transform.translation.x;
        M(1, 3) = (float)trans.transform.translation.y;
        M(2, 3) = (float)trans.transform.translation.z;
        q.x() = (float)trans.transform.rotation.x;
        q.y() = (float)trans.transform.rotation.y;
        q.z() = (float)trans.transform.rotation.z;
        q.w() = (float)trans.transform.rotation.w;
        M.block<3,3>(0,0) = q.toRotationMatrix();

        return M;
    }


    // laser odom
    void laser_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (!laser_ready)
            laser_ready = true;
//        std::cout<<"get laser points "<<std::endl;
        auto s = std::chrono::system_clock::now();
        static int cnt = correct_count_;

        // add
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_raw(new pcl::PointCloud<pcl::PointXYZI>);
        // to pcl points
        pcl::fromROSMsg(*msg, *pcl_cloud);

        ROS_INFO("Transformed point cloud size 1: %lu", pcl_cloud->size());

        if(pcl_cloud->empty()) {
            std::cerr<<"cloud is empty"<<std::endl;
            return;
        }
        // pcl_cloud->points.reserve(pcl_raw->points.size());
        // for (const auto &pt:pcl_raw->points){
        //     if (pt.z > -0.4)
        //     {
        //             pcl_cloud->points.emplace_back(pt);
        //     }
        // }

        //std::cout<<"size: "<<filter->points.size()<<std::endl;
        // tf to base frame
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, laser_base_);

        //输出pcl_cloud大小
//        ROS_INFO("Transformed point cloud size 2: %lu", pcl_cloud->size());
//        ROS_INFO("Transformed point cloud size 2: %lu", pcl_cloud->width);

        Eigen::Matrix4f current_pos;
        // normal localization
        //if ((localization_state == 0 && correct_frames.empty()) || localization_state == 1)
        {
            Eigen::Matrix4f odom;
            if (location_mode_ == 1) {
//                ROS_INFO("Transformed point cloud size 3: %lu", pcl_cloud->size());
                odom = odometry->addFrame(pcl_cloud);
                odom = odom * laser_base_;
                // 发布定位信息
                geometry_msgs::TransformStamped t;
                t.header.stamp = ros::Time::now();;
                t.header.frame_id = odom_frame_;
                t.child_frame_id = base_link_frame_;
                geometry_msgs::Transform transform;
                Eigen::Quaternionf q1(odom.block<3,3>(0,0));
                transform.translation.x = odom(0, 3);
                transform.translation.y = odom(1, 3);
                transform.translation.z = odom(2, 3);
                transform.rotation.x = q1.x();
                transform.rotation.y = q1.y();
                transform.rotation.z = q1.z();
                transform.rotation.w = q1.w();
                t.transform = transform;
                // Send the transformation
                tf_broadcaster_->sendTransform(t);

                // 添加轨迹线
                viewer_->addLine(pcl::PointXYZ(last_pose_(0, 3), last_pose_(1, 3), last_pose_(2, 3)),
                                 pcl::PointXYZ(odom(0, 3), odom(1, 3), odom(2, 3)), std::to_string(counter_++));
                // 更新位姿
                last_pose_ = odom;
                //输出位姿
                ROS_INFO("odom: %f %f %f", odom(0, 3), odom(1, 3), odom(2, 3));
                // 更新可视化
                pcl::transformPointCloud(*pcl_cloud, *show_, odom);
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> view_h(show_, "intensity");
                viewer_->updatePointCloud(show_, view_h, "show");

                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> view_h2(odometry->getLocalMap(), "intensity");
                viewer_->updatePointCloud(odometry->getLocalMap(), view_h2, "map");
                viewer_->spinOnce();
            }else if (location_mode_ == 2){
                // with odometry
                // 1. get last pos --> current pos
                geometry_msgs::TransformStamped trans;
                while(!buffer_->canTransform(
                        odom_frame_,
                        base_link_frame_, ros::Time(0)));//tf2::TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec) + std::chrono::seconds(msg->header.stamp.sec))));
                try{
//                    ROS_INFO("odom_frame_ 3: %s", odom_frame_.c_str());
//                    ROS_INFO("base_link_frame_ 3: %s", base_link_frame_.c_str());
                    trans = buffer_->lookupTransform(
                            odom_frame_,
                            base_link_frame_, ros::Time(0));//tf2::TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec) + std::chrono::seconds(msg->header.stamp.sec)));
                    current_pos = transform2Matrix(trans);
                }
                catch (tf2::TransformException &ex){
                    ROS_INFO("Could not transform %s", ex.what());
                }
                bool accept = false;
//                std::cout<<"current odom pose: "<<current_pos<<std::endl;
                Eigen::Matrix4f delta_pos = last_trans_.inverse() * current_pos;
                odom = odometry->addFrame(pcl_cloud, delta_pos, accept);
//                std::cout<<"delta_pos "<<delta_pos<<"odom"<<odom<<std::endl;
                last_trans_ = current_pos;
            }

            if (cnt > correct_count_ && correct_frames.size() < 2){
                lock_.lock();
                correct_frames.push(pcl_cloud);
                correct_frames_pose.push(odom);
                if (location_mode_ == 2){
                    odom_pose.emplace(current_pos);
                }
                lock_.unlock();
                cnt = 0;
            }
            cnt ++;
        }
        auto e = std::chrono::system_clock::now();
        //std::cout<<"frame time "<<(float)std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count()/1000.f<<" s"<<std::endl;

        // publish map --> odom
        publish_pos(localization_->get_pos());
//        ROS_INFO("pose_save_ ---: %s", pose_save_.c_str());
        write_position_2(pose_save_, localization_->get_estimate_pos());

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void set_initial_position(){
//        std::string save_txt;
//        this->get_parameter("initial_pose_save", save_txt);
        write_position(initial_pose_save_, localization_->get_estimate_pos());
    }

    pcl::PointCloud<PointT>::Ptr show_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    Eigen::Matrix4f last_pose_;
    int counter_ = 0; // 用于生成唯一的线条ID

    ros::NodeHandle nh_;
    ros::Timer timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    ros::Subscriber subscription_;
    ros::Subscriber laser_sub_;
    ros::Publisher map_pub_;
    ros::Publisher aligned_pub_;
    laser_localization::localization* localization_;
    global_localization::GlobalLocalizationBBS* global_localization_;
    laserOdometry* odometry;

    std::queue<pcl::PointCloud<PointT>::Ptr> correct_frames;
    std::queue<Eigen::Matrix4f> correct_frames_pose;
    std::queue<Eigen::Matrix4f> odom_pose;
    std::shared_ptr<std::thread> t1;
    std::mutex lock_;
    int location_mode_ = 1;
    int localization_state = 0;
    int correct_count_ = 10;
    bool manual_relocalization = false;
    bool odom_ready = false;
    bool laser_ready = false;

    double max_range_, min_tx_, max_tx_, min_ty_, max_ty_, min_theta_, max_theta_;
    double map_min_z_, map_max_z_, map_resolution_;
    int map_width_, map_height_;
    int max_points_pre_cell_, map_pyramid_level_;
    double scan_min_z_, scan_max_z_;
    double map_filter_resolution_, scan_filter_resolution_;
    double global_map_width_, global_map_height_;

    int ndt_max_iterations_, odom_local_map_size_;
    double ndt_resolution_, ndt_step_size_, ndt_epsilon_;
    double ndt_frame_resolution_, odom_kf_distance_, ndt_local_map_resolution_;
    double global_resolution_, global_frame_resolution_, global_view_resolution_;
    double local_map_size_;

    double localization_ndt_resolution_, localization_ndt_step_size_, localization_ndt_epsilon_;

    std::string global_map_, odom_frame_, base_link_frame_, laser_frame_;
    std::string laser_topic_, pose_save_, initial_pose_save_;

    ros::ServiceServer initial_service_;
    ros::ServiceServer relocalization_service_;

    Eigen::Matrix4f last_trans_;
    Eigen::Matrix4f laser_base_;


//    std::string odom_frame = "odom";
//    std::string base_link_frame = "base_link";
//    std::string laser_frame = "os_sensor";
//
//    Eigen::Matrix4f last_trans_;
//    Eigen::Matrix4f laser_base_;
//    global_localization::GlobalLocalizationBBS* global_localization_;
//    laserOdometry* odometry;
//    rclcpp::TimerBase::SharedPtr timer_;
//    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//    std::shared_ptr<tf2_ros::Buffer> buffer_;
//    // 订阅cmd,用于测试
//    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
//    // 订阅激光雷达数据
//    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_sub_;
//
//    // 订阅里程计数据
//    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//    // 获取激光雷达与里程计的相对位置关系
//    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
//    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
//    laser_localization::localization* localization_;
//
//    std::queue<pcl::PointCloud<PointT>::Ptr> correct_frames;
//    std::queue<Eigen::Matrix4f> correct_frames_pose;
//    std::queue<Eigen::Matrix4f> odom_pose;
//    std::shared_ptr<std::thread> t1;
//    std::mutex lock_;
//    // 用于调试模式，利用cmd_vel指令推测当前位置，用于测试路径规划算法
//    int location_mode_ = 1;
//
//    // 0 wait initial
//    // 1 normal
//    // 2 error
//    int localization_state = 0;
//    int correct_count = 10;
//
//    bool manual_relocalization = false;
//
//    // service
//    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr initial_service_;
//    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr relocalization_service_;
//
//    bool odom_ready = false;
//    bool laser_ready = false;
};

// read msg then publish to ros2
int main(int argc, char * argv[])
{
//    rclcpp::init(argc, argv);
//    auto node = std::make_shared<Publisher>();
//    rclcpp::spin(node);
//    rclcpp::shutdown();
    ros::init(argc, argv, "my_localization");
    ros::NodeHandle nh;
    Publisher pub(nh);
    ros::spin();
    return 0;
}





