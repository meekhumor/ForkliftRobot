source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
Terminal 1


source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
Terminal 2 


source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
Terminal 3



source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/vishwa/Desktop/ForkliftRobot/maps/forklift_map.yaml \
  params_file:=/home/vishwa/Desktop/ForkliftRobot/config/navigation/nav2_params.yaml \
  use_sim_time:=False
Terminal 4 



source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run forklift_bridge velocity_bridge \
Terminal 5


cd ~/Desktop/ForkliftRobot
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 dashboard/app.py
Terminal 6 (flask dashboard)
