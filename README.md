
# SIMULTANEOUS LOCALIZATION AND MAPPING (SLAM)

This repo contains the code developed through my master's dissertation (avaiable [here](https://github.com/wvmcastro/dissertacao-mestrado/blob/main/tese.pdf), for portuguese readers). It has ROS compatible implementations for **EKF-SLAM**, **EIF-SLAM** and **SEIF-SLAM**, the later being my main object of study in scenarios with more than one robot (distributed and descentralized SLAM).

## Demonstration

### EKF-SLAM and EIF-SLAM
In this comparison video the robot was teleoperated in gazebo and the control inputs were recorded in a rosbag file, then both EKF-SLAM and EIF-SLAM algorithms were both fed with these inputs. The red path is the odometry (encoders) estimation, yellow the filters estimation and the real path is in green.

[![Watch the video](https://user-images.githubusercontent.com/12619298/232341967-4dd16944-a9e9-4d41-ac31-15cfefebd8fe.png)](https://www.youtube.com/watch?v=X6Tk1kcKhoI)

### SEIF-SLAM
Gazebo and RViz view of teleoperated robot running SEIF-SLAM estimation, with active landmarks set of size 4
[![Watch the video](https://user-images.githubusercontent.com/12619298/232776794-fc6a168b-2801-491c-92a4-f5b5e6302ce4.png)](https://www.youtube.com/watch?v=EpC5KOj4ka0)

---
## TODO
 - [ ] Videos
 - [ ] Docker image 
 - [ ] CLI examples
