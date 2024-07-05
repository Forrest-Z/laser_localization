#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "datasetReader.h"

using namespace std::chrono_literals;
class Publisher
{
public:
    Publisher(ros::NodeHandle& nh)
            :nh_(nh)
    {
        DatasetOptions options;
        options.root_path = "/home/vio/Datasets/KITTI/KITTI_raw";
        options.dataset = KITTI_raw;
        options.fail_if_incomplete = false;
        options.min_dist_lidar_center = 5.0;
        options.max_dist_lidar_center = 100.0;

        iterator_ptr_ = get_dataset_sequence(options, 0);
        publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);

        timer_ = nh.createTimer(ros::Duration(0.2), boost::bind(&Publisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (iterator_ptr_->HasNext()) {
            std::vector<Point3D> frame = iterator_ptr_->Next();
            sensor_msgs::PointCloud2 map;
            map.header.stamp = ros::Time::now();
            map.header.frame_id = "base_link";
            map.height = 1;
            map.width = frame.size();
            map.is_bigendian = false;
            map.is_dense = false;

            // 设置 PointCloud2 消息的字段
            sensor_msgs::PointCloud2Modifier modifier(map);
            modifier.setPointCloud2FieldsByString(1, "xyz");

            // 设置 PointCloud2 数据的大小
            map.data.resize(frame.size() * sizeof(float) * 4);
            map.point_step = sizeof(float) * 4;
            map.row_step = map.point_step * map.width;

            // 填充 PointCloud2 数据
            sensor_msgs::PointCloud2Iterator<float> iter_x(map, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(map, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(map, "z");

            for (const auto&mp:frame) {
                *iter_x = static_cast<float>(mp.raw_pt.x());
                *iter_y = static_cast<float>(mp.raw_pt.y());
                *iter_z = static_cast<float>(mp.raw_pt.z());

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
            publisher_.publish(map);
        }
    }

    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher publisher_;
    std::shared_ptr<datasetSequence> iterator_ptr_;
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;
    Publisher pub(nh);
    ros::spin();
    return 0;
}






