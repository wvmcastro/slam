# NUSLAM

## This package implements EKF-SLAM with turtlebot3.

### Dependencies
* Eigen3
* Matplotlib

### ROS Package Dependencies
* rospy
* roscpp
* nuturtle_gazebo (mine)
* turtlebot3_teleop
* message_genration
* catch_ros

## Executables in this package
### Draw Map
This program reads the landmarks topic specified through a ros parameter and publishes a visualization marker message in the **visualization_marker** topic. So the landmarks can be visualized using Rviz.
### Analysis
The analysis program reads the positions of the landmarks (cylinders) from gazebo and publishes them in the **real/landmarks** namespace, in the **visualization_marker_array** topic. So they can be visualized usin Rviz.
### Fake Landmarks
This program also reads the landmarks positions from gazebo and publishes it on the **landmarks** topic. But it "filters" the published landmarks accordingly with a specified radius. It also introduces a adjustable noise in the fake detections. Both the radius and the noise level are specified as ros parameters.

## Launch Files
### landmarks.launch
This file spawns the turtlebot3 burger from the **nuturtle_gazebo** package in the turtlebot3_world world. The landmaks node reads data from the **/scan** topic and publishes the landmarks in the **nuslam/landmarks** topic. Launchs Rviz to visualize the tf frames, odometry and cylinder markers (representing the detected landmarks). 
The **landmarks.launch** file has a ***debug*** parameter (false by default). When true launchs the **landmarks_debug** node that shows **nuslam/LandmarkDebug.msg** messages data using matplotlib.
