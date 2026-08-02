# 轨迹点（对齐 Autoware TrajectoryPoint 精简版）
builtin_interfaces/Duration time_from_start
geometry_msgs/Pose pose                        # odom 系
float32 longitudinal_velocity_mps
float32 acceleration_mps2
float32 curvature                              # [1/m]，左转正
