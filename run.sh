source install/setup.sh
export ROS_DOMAIN_ID=$(expr $1 + 1)
ros2 launch simulation_bringup simulation.launch.py robot_id:=$1