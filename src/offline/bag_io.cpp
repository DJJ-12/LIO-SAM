#include "lio_sam/offline/bag_io.hpp"

#include <chrono>
#include <iostream>
#include <set>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace lio_sam::offline {
namespace {

std::string StripLeadingSlash(const std::string& topic) {
    size_t i = 0;
    while (i < topic.size() && topic[i] == '/') ++i;
    return topic.substr(i);
}

std::string WithLeadingSlash(const std::string& topic) {
    if (topic.empty()) return topic;
    if (topic.front() == '/') return topic;
    return "/" + topic;
}

std::vector<std::string> TopicAliases(const std::string& topic) {
    std::set<std::string> aliases;
    aliases.insert(topic);

    const std::string no_slash = StripLeadingSlash(topic);
    if (!no_slash.empty()) {
        aliases.insert(no_slash);
        aliases.insert("/" + no_slash);
    }

    // Some converted DDS bags may keep DDS-style prefixes.
    if (no_slash.rfind("rt/", 0) == 0 && no_slash.size() > 3) {
        const std::string no_rt = no_slash.substr(3);
        aliases.insert(no_rt);
        aliases.insert("/" + no_rt);
    }
    if (no_slash.rfind("rq/", 0) == 0 && no_slash.size() > 3) {
        const std::string no_rq = no_slash.substr(3);
        aliases.insert(no_rq);
        aliases.insert("/" + no_rq);
    }
    if (no_slash.rfind("rr/", 0) == 0 && no_slash.size() > 3) {
        const std::string no_rr = no_slash.substr(3);
        aliases.insert(no_rr);
        aliases.insert("/" + no_rr);
    }

    return std::vector<std::string>(aliases.begin(), aliases.end());
}

}  // namespace

RosbagIO::RosbagIO(std::string bag_file, std::string storage_id)
    : bag_file_(std::move(bag_file)), storage_id_(std::move(storage_id)) {}

RosbagIO& RosbagIO::AddHandle(const std::string& topic_name, MessageProcessFunction func) {
    for (const auto& alias : TopicAliases(topic_name)) {
        process_func_[alias] = func;
    }
    registered_topics_.insert(topic_name);
    return *this;
}

RosbagIO& RosbagIO::AddPointCloud2Handle(const std::string& topic_name, PointCloud2Handle func) {
    return AddHandle(topic_name, [this, func = std::move(func)](const MsgType& m) -> bool {
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        rclcpp::SerializedMessage data(*m->serialized_data);
        seri_cloud2_.deserialize_message(&data, msg.get());
        return func(msg);
    });
}

RosbagIO& RosbagIO::AddImuHandle(const std::string& topic_name, ImuHandle func) {
    return AddHandle(topic_name, [this, func = std::move(func)](const MsgType& m) -> bool {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        rclcpp::SerializedMessage data(*m->serialized_data);
        seri_imu_.deserialize_message(&data, msg.get());
        return func(msg);
    });
}

RosbagIO& RosbagIO::AddOdometryHandle(const std::string& topic_name, OdomHandle func) {
    return AddHandle(topic_name, [this, func = std::move(func)](const MsgType& m) -> bool {
        auto msg = std::make_shared<nav_msgs::msg::Odometry>();
        rclcpp::SerializedMessage data(*m->serialized_data);
        seri_odom_.deserialize_message(&data, msg.get());
        return func(msg);
    });
}

RosbagIO& RosbagIO::AddLoopHandle(const std::string& topic_name, LoopHandle func) {
    return AddHandle(topic_name, [this, func = std::move(func)](const MsgType& m) -> bool {
        auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        rclcpp::SerializedMessage data(*m->serialized_data);
        seri_loop_.deserialize_message(&data, msg.get());
        return func(msg);
    });
}

void RosbagIO::CleanProcessFunc() {
    process_func_.clear();
    registered_topics_.clear();
}

void RosbagIO::Go(int sleep_usec) {
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file_;
    storage_options.storage_id = storage_id_;
    rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
    reader.open(storage_options, converter_options);

    std::cout << "[OFFLINE_BAG] registered topics:" << std::endl;
    for (const auto& t : registered_topics_) {
        std::cout << "  - " << t << std::endl;
    }

    std::cout << "[OFFLINE_BAG] bag topics:" << std::endl;
    for (const auto& topic_type : reader.get_all_topics_and_types()) {
        std::cout << "  - " << topic_type.name << " [" << topic_type.type << "]" << std::endl;
    }

    size_t total = 0;
    size_t used = 0;
    std::map<std::string, size_t> total_by_topic;
    std::map<std::string, size_t> used_by_topic;

    while (rclcpp::ok() && reader.has_next()) {
        auto msg = reader.read_next();
        ++total;
        ++total_by_topic[msg->topic_name];

        auto iter = process_func_.find(msg->topic_name);
        if (iter == process_func_.end()) {
            for (const auto& alias : TopicAliases(msg->topic_name)) {
                iter = process_func_.find(alias);
                if (iter != process_func_.end()) break;
            }
        }

        if (iter != process_func_.end()) {
            ++used;
            ++used_by_topic[msg->topic_name];
            if (!iter->second(msg)) {
                break;
            }
        }
        if (sleep_usec > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_usec));
        }
    }

    std::cout << "[OFFLINE_BAG] dispatch stats:" << std::endl;
    for (const auto& kv : total_by_topic) {
        const auto u = used_by_topic.count(kv.first) ? used_by_topic[kv.first] : 0;
        std::cout << "  - " << kv.first << ": total=" << kv.second << ", dispatched=" << u << std::endl;
    }

    std::cout << "[OFFLINE_BAG] finished: total messages=" << total
              << ", dispatched messages=" << used << std::endl;
}

}  // namespace lio_sam::offline
