#include <chrono>
#include <functional>
#include <memory>
#include <queue>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "localization/localization.h"
#include "laser_odometry/ct_odometry.h"
#include "datasetReader.h"

#include "unistd.h"
#include "thread"
#include "fstream"

#include <boost/bind.hpp>

using namespace std::chrono_literals;

class localization
{
public:
    explicit localization(ros::NodeHandle& nh);
private:

    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher frame_pub_;
    ros::Publisher map_pub_;
    ros::Publisher local_map_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    ros::Subscriber laser_sub_;

    std::shared_ptr<laser_localization::localization> localization_;
    std::shared_ptr<Odometry> odometry_;
    Odometry::RegistrationSummary odometry_result_;
    std::shared_ptr<std::thread> odom_thread_;

    // data
    std::queue<std::vector<Point3D>> laser_buffer_;
    std::mutex laser_buffer_lock_;
    laser_localization::localization_options l_options;
    OdometryOptions o_options;

    void timer_callback();
    void odom_thread();
    void laser_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void publish_points(const ros::Publisher& pub, std::string frame_id, const std::vector<Point3D> &points);
    void publish_odom(Eigen::Matrix3d R, Eigen::Vector3d t);
    void publish_voxel_map(const ros::Publisher& pub, std::string frame_id, const VoxelHashMap& map);
};

localization::localization(ros::NodeHandle &nh)
        :nh_(nh)
{

    nh_.param<std::string>("localization/global_map_path", l_options.global_map_path,"/home/vio/Code/RobotSystem/human/src/localization/laser_localization/map/kitti.ply");
    nh_.param<double>("localization/filter_k", l_options.filter_k, 0.1);
    nh_.param<int>("localization/frame_gap", l_options.frame_gap, 30);
    nh_.param<int>("localization/frame_size", l_options.frame_size, 50);

    nh_.param<double>("odometry/voxel_size", o_options.voxel_size, 0.5);
    nh_.param<double>("odometry/sample_voxel_size", o_options.sample_voxel_size, 1.5);
    nh_.param<double>("odometry/max_distance", o_options.max_distance, 100.0);
    nh_.param<int>("odometry/max_num_points_in_voxel", o_options.max_num_points_in_voxel, 20);
    nh_.param<bool>("odometry/debug_print", o_options.debug_print, false);
    nh_.param<int>("odometry/init_num_frames", o_options.init_num_frames, 20);
    nh_.param<bool>("odometry/robust_registration", o_options.robust_registration, false);
    nh_.param<double>("odometry/min_distance_points", o_options.min_distance_points, 0.1);
    nh_.param<double>("odometry/distance_error_threshold", o_options.distance_error_threshold, 5.0);
    int motion_compensation,  initialization;
    nh_.param<int>("odometry/motion_compensation", motion_compensation, CONTINUOUS);
    nh_.param<int>("odometry/initialization", initialization, INIT_CONSTANT_VELOCITY);
    o_options.motion_compensation = static_cast<MOTION_COMPENSATION>(motion_compensation);
    o_options.initialization = static_cast<INITIALIZATION>(initialization);

    nh_.param<double>("ct_icp/size_voxel_map", o_options.ct_icp_options.size_voxel_map, 1.0);
    nh_.param<int>("ct_icp/num_iters_icp", o_options.ct_icp_options.num_iters_icp, 5);
    nh_.param<int>("ct_icp/min_number_neighbors", o_options.ct_icp_options.min_number_neighbors, 20);
    int voxel_neighborhood;
    nh_.param<int>("ct_icp/voxel_neighborhood", voxel_neighborhood, 1);
    o_options.ct_icp_options.voxel_neighborhood = (short)voxel_neighborhood;
    nh_.param<int>("ct_icp/max_number_neighbors", o_options.ct_icp_options.max_number_neighbors, 20);
    nh_.param<double>("ct_icp/max_dist_to_plane_ct_icp", o_options.ct_icp_options.max_dist_to_plane_ct_icp, 0.3);
    nh_.param<bool>("ct_icp/debug_print", o_options.ct_icp_options.debug_print, false);
    nh_.param<bool>("ct_icp/point_to_plane_with_distortion", o_options.ct_icp_options.point_to_plane_with_distortion, true);
    int distance;
    nh_.param<int>("ct_icp/distance", distance, CT_POINT_TO_PLANE);
    o_options.ct_icp_options.distance = static_cast<ICP_DISTANCE>(distance);
    nh_.param<int>("ct_icp/num_closest_neighbors", o_options.ct_icp_options.num_closest_neighbors, 1);
    nh_.param<double>("ct_icp/beta_location_consistency", o_options.ct_icp_options.beta_location_consistency, 0.001);
    nh_.param<double>("ct_icp/beta_constant_velocity", o_options.ct_icp_options.beta_constant_velocity, 0.001);

    int solver, loss_function;
    nh_.param<int>("ct_icp/solver", solver, GN);
    nh_.param<int>("ct_icp/loss_function", loss_function, HUBER);
    o_options.ct_icp_options.solver = static_cast<CT_ICP_SOLVER>(solver);
    o_options.ct_icp_options.loss_function = static_cast<LEAST_SQUARES>(loss_function);

    nh_.param<int>("ct_icp/ls_max_num_iters", o_options.ct_icp_options.ls_max_num_iters, 1);
    nh_.param<int>("ct_icp/ls_num_threads", o_options.ct_icp_options.ls_num_threads, 16);

    nh_.param<double>("ct_icp/ls_sigma", o_options.ct_icp_options.ls_sigma, 0.1);
    nh_.param<double>("ct_icp/ls_tolerant_min_threshold", o_options.ct_icp_options.ls_tolerant_min_threshold, 0.05);

    std::string laser_topic;
    nh_.param<std::string>("laser_topic", laser_topic, "/velodyne_points");

    localization_ = std::make_shared<laser_localization::localization>(l_options);
    odometry_ = std::make_shared<Odometry>(&o_options);

    // publisher
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map", 10);
    local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 10);
    frame_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/frame", 10);
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    // visual timer
    timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&localization::timer_callback, this));
    // laser odom
    odom_thread_ = std::make_shared<std::thread>(&localization::odom_thread, this);

    // subscribe laser
    laser_sub_ = nh.subscribe(laser_topic, 10, &localization::laser_callback, this);
}

// 10Hz
void localization::timer_callback() {
    // tmp
    {
        DatasetOptions options;
        options.root_path = "/home/vio/Datasets/KITTI/KITTI_raw";
        options.dataset = KITTI_raw;
        options.fail_if_incomplete = false;
        options.min_dist_lidar_center = 5.0;
        options.max_dist_lidar_center = 100.0;

        static std::shared_ptr<datasetSequence> iterator_ptr_ = get_dataset_sequence(options, 0);

        if (iterator_ptr_->HasNext()) {
            std::vector<Point3D> frame = iterator_ptr_->Next();
            laser_buffer_lock_.try_lock();
            laser_buffer_.push(frame);
            laser_buffer_lock_.unlock();
            publish_points(frame_pub_, "base_link", frame);
        }
    }
    static int delay_cnt = 0;
    delay_cnt ++;
    if (delay_cnt > 10) {delay_cnt = 0;}

    if (delay_cnt == 0) {// 1. publish global map 1hz
        publish_points(map_pub_, "map", localization_->global_map());
    } else if (delay_cnt == 5) {// 2. publish local map 1hz
        publish_voxel_map(local_map_pub_, "odom", odometry_->GetVoxelMap());
    }
    // 2. publish tf
    if (!odometry_result_.all_corrected_points.empty()) {
        publish_odom(odometry_result_.frame.begin_R, odometry_result_.frame.begin_t);
    }
}

void localization::laser_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::vector<Point3D> frame;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Point3D point;
        point.raw_pt[0] = *iter_x;
        point.raw_pt[1] = *iter_y;
        point.raw_pt[2] = *iter_z;

        point.pt = point.raw_pt;
        point.alpha_timestamp = 0;
        ros::Time current_time = ros::Time::now();
        point.timestamp = double(current_time.sec * 1000000000ULL + current_time.nsec);
        frame.emplace_back(point);
    }

    laser_buffer_lock_.try_lock();
    laser_buffer_.push(frame);
    laser_buffer_lock_.unlock();
}

void localization::publish_points(const ros::Publisher& pub, std::string frame_id, const std::vector<Point3D> &points) {
    sensor_msgs::PointCloud2 map;
    map.header.stamp = ros::Time::now();
    map.header.frame_id = frame_id;
    map.height = 1;
    map.width = points.size();
    map.is_bigendian = false;
    map.is_dense = false;

    // 设置 PointCloud2 消息的字段
    sensor_msgs::PointCloud2Modifier modifier(map);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // 设置 PointCloud2 数据的大小
    map.data.resize(points.size() * sizeof(float) * 3);
    map.point_step = sizeof(float) * 3;
    map.row_step = map.point_step * map.width;

    // 填充 PointCloud2 数据
    sensor_msgs::PointCloud2Iterator<float> iter_x(map, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(map, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(map, "z");

    for (const auto&mp:points) {
        *iter_x = static_cast<float>(mp.raw_pt.x());
        *iter_y = static_cast<float>(mp.raw_pt.y());
        *iter_z = static_cast<float>(mp.raw_pt.z());

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    pub.publish(map);
}

void localization::publish_odom(Eigen::Matrix3d R, Eigen::Vector3d t) {
    geometry_msgs::TransformStamped TransformStamped;
    TransformStamped.header.stamp = ros::Time::now();
    TransformStamped.header.frame_id = "odom";
    TransformStamped.child_frame_id = "base_link";

    Eigen::Quaterniond q(R);
    TransformStamped.transform.translation.x = t.x();
    TransformStamped.transform.translation.y = t.y();
    TransformStamped.transform.translation.z = t.z();
    TransformStamped.transform.rotation.x = q.x();
    TransformStamped.transform.rotation.y = q.y();
    TransformStamped.transform.rotation.z = q.z();
    TransformStamped.transform.rotation.w = q.w();

    // Send the transformation
    tf_pub_->sendTransform(TransformStamped);
}

void localization::publish_voxel_map(const ros::Publisher& pub, std::string frame_id, const VoxelHashMap& voxel_map) {
    sensor_msgs::PointCloud2 map;
    map.header.stamp = ros::Time::now();
    map.header.frame_id = frame_id;
    map.height = 1;
    map.width = MapSize(voxel_map);
    map.is_bigendian = false;
    map.is_dense = false;

    // 设置 PointCloud2 消息的字段
    sensor_msgs::PointCloud2Modifier modifier(map);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // 设置 PointCloud2 数据的大小
    map.data.resize(MapSize(voxel_map) * sizeof(float) * 3);
    map.point_step = sizeof(float) * 3;
    map.row_step = map.point_step * map.width;

    // 填充 PointCloud2 数据
    sensor_msgs::PointCloud2Iterator<float> iter_x(map, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(map, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(map, "z");

    for (auto &voxel: voxel_map) {
        for (int i(0); i < voxel.second.NumPoints(); ++i) {

            *iter_x = static_cast<float>(voxel.second.points[i][0]);
            *iter_y = static_cast<float>(voxel.second.points[i][1]);
            *iter_z = static_cast<float>(voxel.second.points[i][2]);

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }
    pub.publish(map);
}

void localization::odom_thread() {
    while (ros::ok()) {
        if (!laser_buffer_.empty()) {
            std::vector<Point3D>& frame = laser_buffer_.front();
            odometry_result_ = odometry_->RegisterFrame(frame);
            std::cout<<" odom: "<<odometry_result_.frame.begin_t<<std::endl;
            laser_buffer_lock_.try_lock();
            laser_buffer_.pop();
            laser_buffer_lock_.unlock();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "my_localization");
    ros::NodeHandle nh;
    localization lo(nh);
    ros::spin();
    return 0;
}






