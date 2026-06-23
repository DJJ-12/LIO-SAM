std_msgs/Header header

# Deskewed full cloud from imageProjection
sensor_msgs/PointCloud2 cloud_deskewed

# Feature clouds from featureExtraction
sensor_msgs/PointCloud2 cloud_corner
sensor_msgs/PointCloud2 cloud_surface

# IMU / odometry availability for initial guess
bool imu_available
bool odom_available

float32 imu_roll_init
float32 imu_pitch_init
float32 imu_yaw_init

float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_roll
float32 initial_guess_pitch
float32 initial_guess_yaw

# Range-image metadata
int32[] start_ring_index
int32[] end_ring_index
int32[] point_col_ind
float32[] point_range
