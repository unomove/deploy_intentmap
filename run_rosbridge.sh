export ZMQ=1
roslaunch roslaunch/floorplan_to_gridmap.launch &
./dist_arm64/ros_bridge/ros_bridge &
# wait ros to be ready
sleep 10
python2 handle_initial.py &