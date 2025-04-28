Running the code:
git clone
colcon build --symlink-install
source install/setup.sh
ros2 launch <package-name> <launch-file>

Lidar only for mapping: mapping.launch.py
Lidar and camera for mapping: camera_mapping.launch.py

localiza_only: False to building the map, True to localiza 
restart_map: create a new map instead of using the current map in .ros/
