# Atlas GPS Driver

This is a ROS2 driver for [Atlas](https://pointonenav.com/docs/atlas) GPS / IMU Devices. 

> The Atlas delivers 10cm location accuracy in any environment with its proprietary [FusionEngine](https://pointonenav.com/fusionengine) software.

### Getting Started

##### Install driver dependencies:
```bash
git clone https://github.com/jimenezjose/atlas_gps_driver.git
cd atlas_gps_driver
sudo apt-get update
sudo rosdep update
rosdep install -y --ignore-src --from-paths atlas_gps_driver/
```

##### Run Atlas node:

* Edit the parameters in `atlas_gps_driver/param/atlas_gps_driver.param.yaml`.
* Optional: Remap output topic names in `atlas_gps_driver/launch/atlas_gps_driver.launch.py`.

```bash
cd atlas_gps_driver
source /opt/ros/galactic/setup.bash
colcon build
source install/local_setup.bash
ros2 launch atlas_gps_driver atlas_gps_driver.launch.py
```
