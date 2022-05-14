# d435_ros


This is a barebones framework to read the depth image from the Intel Realsense D435 and convert it into a ROS LaserScan.

### Steps

1.  Install ROS and dependencies, make a workspace, make a package. [Tutorial](http://wiki.ros.org/ROS/Tutorials)
2.  Install the RealSense ROS library [Tutorial 1](https://github.com/IntelRealSense/realsense-ros), [Tutorial 2](http://wiki.ros.org/realsense_camera). Use `sudo apt-get install ros-<ROS_VERSION>-realsense2-camera` instead of `ros-<ROS_VERSION>-realsense-camera`.
3.  `roslaunch realsense2_camera rs_camera.launch` for the RealSense topics
4.  **Make sure to include the correct python requirements in the CMakeList.txt. The repo version has it included at [Here](https://github.com/christoaluckal/d435_ros/blob/bac3eaae333b0a07f7266159a4188bfc1f1b4b7c/package/CMakeLists.txt#L180).**
5.  Run `rosrun <package-name> laser.py` which is a pub-sub which subscribes to the RealSense depth stream and publishes the central row as a LaserScan.
6.  Run `rosrun <package-name> listener.py` which subscribes to the LaserScan and prints the depths (ranges param for LaserScans)


### Possible Improvements
- [ ] The order in which nodes are created and publish/subscribe are just made to be functional so additional optimization might be possible
- [ ] Values are hardcoded since I have limited knowledge of LaserScans
- [ ] Echoing the nodes seems slow so I'm not sure about performance
