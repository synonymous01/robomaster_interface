# ROS melodic wrapper for the RoboMaster SDK
_made by me solely_

# How to install 
## Dependecies
```
sudo apt-get install ros-melodic-robot-pose-ekf 
```

go to your workspace and clone this repo over there
```
cd catkin_ws/src
git clone https://github.com/synonymous01/robomaster_interface.git
```



# Why did I make this?
The Robomaster SDK works at an intrinsically different python version than ROS Melodic. This was a major issue for my projects in bachelors because we needed to incorporate other sensors with the Robomaster platform. This requires your Jetson Nano to have both Python3.6 and Python2.7 (which are usually pre-installed) 
