#pragma once

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <vector>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace lio_sam::offline {

class RosbagIO {
public:
    using MsgType = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;
    using MessageProcessFunction = std::function<bool(const MsgType&)>;
    using PointCloud2Handle = std::function<bool(sensor_msgs::msg::PointCloud2::SharedPtr)>;
    using ImuHandle = std::function<bool(sensor_msgs::msg::Imu::SharedPtr)>;
    using OdomHandle = std::function<bool(nav_msgs::msg::Odometry::SharedPtr)>;
    using LoopHandle = std::function<bool(std_msgs::msg::Float64MultiArray::SharedPtr)>;

    explicit RosbagIO(std::string bag_file, std::string storage_id = "sqlite3");

    RosbagIO& AddHandle(const std::string& topic_name, MessageProcessFunction func);
    RosbagIO& AddPointCloud2Handle(const std::string& topic_name, PointCloud2Handle func);
    RosbagIO& AddImuHandle(const std::string& topic_name, ImuHandle func);
    RosbagIO& AddOdometryHandle(const std::string& topic_name, OdomHandle func);
    RosbagIO& AddLoopHandle(const std::string& topic_name, LoopHandle func);

    void CleanProcessFunc();
    void Go(int sleep_usec = 0);

private:
    std::map<std::string, MessageProcessFunction> process_func_;
    std::set<std::string> registered_topics_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> seri_cloud2_;
    rclcpp::Serialization<sensor_msgs::msg::Imu> seri_imu_;
    rclcpp::Serialization<nav_msgs::msg::Odometry> seri_odom_;
    rclcpp::Serialization<std_msgs::msg::Float64MultiArray> seri_loop_;

    std::string bag_file_;
    std::string storage_id_;
};

}  // namespace lio_sam::offline
