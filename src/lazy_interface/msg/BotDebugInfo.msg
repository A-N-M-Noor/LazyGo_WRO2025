# Odom values for the robot
float32 pos_x
float32 pos_y
float32 heading

# Debug information for the robot's avoidance system
float32 target_angle 0.0
float32 target_distance 1.0
float32 cast_radius 0.3
float32 danger_distance 0.5
float32[] danger_angles

# Debug information for the lidar system
lazy_interface/LidarTowerInfo[] towers

# Debug information for the object pool 
lazy_interface/GlobalTowerInfo[] towerpool
