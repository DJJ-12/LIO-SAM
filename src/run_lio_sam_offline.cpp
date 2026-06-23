// Offline LIO-SAM runner.
//
// This executable keeps the original LIO-SAM algorithm logic, but replaces the
// online ROS topic chain with direct callbacks driven by rosbag2 messages:
//   rosbag IMU       -> IMUPreintegration + ImageProjection
//   rosbag PointCloud-> ImageProjection -> FeatureExtraction -> mapOptimization
//   mapOptimization odom correction -> IMUPreintegration
//   IMUPreintegration incremental odom -> ImageProjection

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "lio_sam/offline/bag_io.hpp"

#ifndef LIO_SAM_OFFLINE_LIBRARY
#define LIO_SAM_OFFLINE_LIBRARY
#endif
#include "imageProjection.cpp"
#include "featureExtraction.cpp"
#include "mapOptmization.cpp"
#include "imuPreintegration.cpp"

namespace {

struct OfflineOptions {
    std::string input_bag;
    std::string params_file;
    std::string storage_id = "sqlite3";
    std::string save_directory = "./lio_sam_offline_map";
    float save_resolution = 0.0f;
    int sleep_usec = 0;
    int loop_interval = 30;  // run one loop-closure check every N accepted feature frames
    bool disable_loop_closure = false;
};

void PrintUsage(const char* prog) {
    std::cout << "Usage:\n"
              << "  " << prog << " --input_bag <bag_dir> --params_file <params.yaml> [options]\n\n"
              << "Options:\n"
              << "  --storage_id <sqlite3|mcap>       rosbag2 storage id, default sqlite3\n"
              << "  --save_directory <dir>           output map directory, default ./lio_sam_offline_map\n"
              << "  --save_resolution <meters>       output map voxel resolution, 0 keeps original resolution\n"
              << "  --sleep_usec <usec>              optional sleep after each dispatched bag message\n"
              << "  --loop_interval <N>              loop-closure check interval in LiDAR frames, default 30\n"
              << "  --disable_loop_closure           do not call loop closure in offline runner\n";
}

bool ParseArgs(int argc, char** argv, OfflineOptions& opts) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need_value = [&](const std::string& key) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << key << std::endl;
                std::exit(2);
            }
            return argv[++i];
        };

        if (a == "--input_bag" || a == "-i") {
            opts.input_bag = need_value(a);
        } else if (a == "--params_file" || a == "--config" || a == "-c") {
            opts.params_file = need_value(a);
        } else if (a == "--storage_id") {
            opts.storage_id = need_value(a);
        } else if (a == "--save_directory" || a == "--output" || a == "-o") {
            opts.save_directory = need_value(a);
        } else if (a == "--save_resolution") {
            opts.save_resolution = std::stof(need_value(a));
        } else if (a == "--sleep_usec") {
            opts.sleep_usec = std::stoi(need_value(a));
        } else if (a == "--loop_interval") {
            opts.loop_interval = std::stoi(need_value(a));
        } else if (a == "--disable_loop_closure") {
            opts.disable_loop_closure = true;
        } else if (a == "--help" || a == "-h") {
            PrintUsage(argv[0]);
            return false;
        }
    }

    if (opts.input_bag.empty()) {
        std::cerr << "[OFFLINE] --input_bag is required." << std::endl;
        PrintUsage(argv[0]);
        return false;
    }
    return true;
}

std::vector<std::string> BuildRosArgs(const char* prog, const OfflineOptions& opts) {
    std::vector<std::string> args{prog};
    if (!opts.params_file.empty()) {
        args.push_back("--ros-args");
        args.push_back("--params-file");
        args.push_back(opts.params_file);
    }
    return args;
}

std::vector<char*> ToArgv(std::vector<std::string>& args) {
    std::vector<char*> out;
    out.reserve(args.size());
    for (auto& s : args) out.push_back(s.data());
    return out;
}

template <typename MsgT>
std::shared_ptr<MsgT> CopyMsg(const MsgT& msg) {
    return std::make_shared<MsgT>(msg);
}

}  // namespace

int main(int argc, char** argv) {
    OfflineOptions opts;
    if (!ParseArgs(argc, argv, opts)) {
        return opts.input_bag.empty() ? 2 : 0;
    }

    auto ros_args = BuildRosArgs(argv[0], opts);
    auto ros_argv = ToArgv(ros_args);
    int ros_argc = static_cast<int>(ros_argv.size());
    rclcpp::init(ros_argc, ros_argv.data());

    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);

    auto image_projection = std::make_shared<ImageProjection>(node_options);
    auto feature_extraction = std::make_shared<FeatureExtraction>(node_options);
    auto map_optimization = std::make_shared<mapOptimization>(node_options);
    auto imu_preintegration = std::make_shared<IMUPreintegration>(node_options);

    if (opts.disable_loop_closure) {
        map_optimization->loopClosureEnableFlag = false;
    }

    size_t lidar_cloud_count = 0;
    size_t deskewed_count = 0;
    size_t feature_count = 0;
    size_t map_odom_count = 0;
    size_t imu_count = 0;
    size_t imu_odom_count = 0;

    // Direct offline wiring. These callbacks are the offline replacement for the
    // online ROS pubs/subs between the original four LIO-SAM nodes.
    image_projection->SetOfflineCloudInfoCallback(
        [&](const lio_sam::msg::CloudInfo& cloud_info) {
            ++deskewed_count;
            feature_extraction->laserCloudInfoHandler(CopyMsg(cloud_info));
        });

    feature_extraction->SetOfflineCloudInfoCallback(
        [&](const lio_sam::msg::CloudInfo& cloud_info) {
            ++feature_count;
            map_optimization->laserCloudInfoHandler(CopyMsg(cloud_info));
            if (!opts.disable_loop_closure && opts.loop_interval > 0 &&
                (feature_count % static_cast<size_t>(opts.loop_interval) == 0)) {
                map_optimization->RunOfflineLoopClosureOnce();
            }
        });

    map_optimization->SetOfflineOdometryCallback(
        [&](const nav_msgs::msg::Odometry& odom) {
            ++map_odom_count;
            imu_preintegration->odometryHandler(CopyMsg(odom));
        });

    imu_preintegration->SetOfflineImuOdometryCallback(
        [&](const nav_msgs::msg::Odometry& odom) {
            ++imu_odom_count;
            image_projection->odometryHandler(CopyMsg(odom));
        });

    lio_sam::offline::RosbagIO bag(opts.input_bag, opts.storage_id);
    bag.AddImuHandle(image_projection->imuTopic,
                     [&](sensor_msgs::msg::Imu::SharedPtr imu) {
                         ++imu_count;
                         image_projection->imuHandler(imu);
                         imu_preintegration->imuHandler(imu);
                         return rclcpp::ok();
                     })
       .AddPointCloud2Handle(image_projection->pointCloudTopic,
                             [&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                 ++lidar_cloud_count;
                                 image_projection->cloudHandler(cloud);
                                 return rclcpp::ok();
                             })
       .AddOdometryHandle(map_optimization->gpsTopic,
                          [&](nav_msgs::msg::Odometry::SharedPtr gps) {
                              map_optimization->gpsHandler(gps);
                              return rclcpp::ok();
                          })
       .AddLoopHandle("lio_loop/loop_closure_detection",
                      [&](std_msgs::msg::Float64MultiArray::SharedPtr loop_msg) {
                          map_optimization->loopInfoHandler(loop_msg);
                          return rclcpp::ok();
                      });

    std::cout << "[OFFLINE] input_bag=" << opts.input_bag << std::endl;
    std::cout << "[OFFLINE] pointCloudTopic=" << image_projection->pointCloudTopic
              << ", imuTopic=" << image_projection->imuTopic
              << ", gpsTopic=" << map_optimization->gpsTopic << std::endl;

    bag.Go(opts.sleep_usec);

    // Run a few final loop-closure passes after all keyframes have been inserted.
    if (!opts.disable_loop_closure) {
        for (int i = 0; i < 3; ++i) {
            map_optimization->RunOfflineLoopClosureOnce();
        }
    }

    bool saved = map_optimization->SaveMapOffline(opts.save_directory, opts.save_resolution);

    std::cout << "[OFFLINE] summary: imu=" << imu_count
              << ", imu_odom=" << imu_odom_count
              << ", lidar_raw=" << lidar_cloud_count
              << ", deskewed=" << deskewed_count
              << ", feature=" << feature_count
              << ", map_odom=" << map_odom_count
              << ", keyframes=" << map_optimization->GetOfflineKeyframeCount()
              << std::endl;
    std::cout << "[OFFLINE] save " << (saved ? "success" : "failed/skipped")
              << ": " << opts.save_directory << std::endl;

    rclcpp::shutdown();
    return saved ? 0 : 3;
}
