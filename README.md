# Submarine Robotics Introduction
This repository is used for a submarine robot course offered at California State University, Los Angeles. The code works with a small submarine robot designed for this courses.
# What does the robot look like?
(1) Robot Configuration: This robot has a confuguration as follows. It has in total 6 thrusters with four (1-4) for leveling and depth control, and two (5-6) for horizontal motion control (forward, backward, turning). 
                    
                    //\\
                    \\// (5)
            //\\     ||     //\\
            \\// (1) ||     \\//  (2)
           ====================
            xxxx             xx
            xxxx             xx  
           ==================== 
            //\\ (3) ||     //\\  (4)
            \\//     ||     \\//
                    //\\
                    \\// (6)
(2) Sensors: Camera, IMU, Temperature sensor  
(3) Mother Board - Jetson Nando (run the overall program)  
(4) Microcontroller - Arduino Uno (for thruster command execution) 
# What's included in this repository?
It provides a miminum system that allows students to install on the onboard computer. The system is programed by using Python with ROS architecture. This minimum system that allows the robot to perform some basic movement: dive 1m -> rotate a 360 degs -> move forward following a sinsoidal curve -> return to the origin.   
# How to use it for the course?
Follow the progress of the course to modify the code within sections with place holders as follows  
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    
#~~~~~~~~ Update Your Code Here ~~~~~~~~  
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
# Table of contents
1. Prepare your onboard computer  
        1.1 install ubuntu 18.04 (pick the version that match Ros versions)
        1.2 install ROS melodic  
        1.3 install git (optional for code backups)  
        1.4 install visual studio (as code editor)  
2. Demo: sensor reading  
        2.1 install Arduino IDE  
        2.2 wirte the code to read sensor data  
        2.3 use jetson to read data from arduino microcontroller  
3. Demo: control system demo  
        3.1 pid depth control on microcontroller
        3.2 position control


# Topic 1: get started
## 1.1 prepare the micro-sd card for Nvidia Jetson Nano   

Follow this link to setup the jetson-nano 4G: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit.  
Follow this link to setup the jetson-nano 2G: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-2gb-devkit.  
Note: For Jetson-nano-2gb board, choose this version: Jetson Nano 2GB Developer Kit SD Card Image 4.6
Username and password choose: robosub
After installation, your system will have this version: Ubuntu 18.04  

---
## 1.2 install ROS  
We will ros melodic as an example, check compatability before choosing which version of ros to install. The official operating system for the Jetson Nano and other Jetson boards is called Linux4Tegra, which is actually a version of Ubuntu 18.04 that's designed to run on Nvidia's hardware. Check this link for details on how to install ros: http://wiki.ros.org/melodic/Installation/Ubuntu.   

Alternative: Follow the following isntruction to install ROS

1. sudo apt-get update
2. sudo apt-get -y install nano
3. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
4. sudo apt install curl
5. curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
6. sudo apt update
7. sudo apt install ros-melodic-desktop
8. echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
9. source ~/.bashrc
10. sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
11. sudo apt install python-rosdep
12. sudo rosdep init
13. rosdep update  

&emsp;**Check if ROS is Installed:**   
&emsp;&emsp; printenv | grep ROS

---
## 1.3 Install Arduino IDE  
Check this link for details: https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview.  
1. cd ~
2. wget https://downloads.arduino.cc/arduino-1.8.19-linuxaarch64.tar.xz
3. tar -xf arduino-1.8.19-linuxaarch64.tar.xz
4. cd arduino-1.8.19/
5. sudo bash install.sh
6. sudo apt-get install ros-melodic-rosserial-arduino
7. sudo apt-get install ros-melodic-rosserial
8. rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
9. *Open Arduino IDE on Desktop:*
	* *In Library Manager Install:*
		* "PID" by Brett Beauregard
		* "MS5837" by Blue Robotics
		* "BNO055" by Adafruit **+ Sub-Libraries**


---
## 1.4 Install Visual Studio Code  
1. Download Linux Arm 64 bits debian here: https://code.visualstudio.com/download  
Instructions: https://code.visualstudio.com/docs/?dv=linuxarm64  

2. cd Downloads/  
3. sudo apt install ./code_1.70.1-1660111764_arm64.deb

To run Visual Studio Code, and setup github call it in a terminal  
4. code  

---
# Topic 2: Install the Repository and Packages
## 2.1 Clone and build the reppository 
1. mkdir ~/ai_class_ws/
2. cd ~/ai_class_ws  
3. git clone https://github.com/Robotics-Courses/Autonomous_Robotics_Class.git 
4. mv Autonomous_Robotics_Class src/
5. catkin_make  
6. echo "source ~/ai_class_ws/devel/setup.bash" >> ~/.bashrc
7. source ~/.bashrc

---
## 2.2 Install Darknet & YOLOv4-Tiny

1. cd ~/ai_class_ws/src/
2. git clone https://github.com/AlexeyAB/darknet.git 
3. cd darknet/
4. export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}}
5. export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
6. nano Makefile
	* Set GPU --> 1
	* Set CUDNN --> 1
	* Set OpenCV --> 1
	* Set LIBSO --> 1
7. make
8. cp libdarknet.so ~/ai_class_ws/src/computer_vision/yolov4_files/
9. nano ../computer_vision/yolov4_files/camerabox.data
	* *Change “names=/home/####/ai_class_ws/...” with your username*

&emsp;**Check if Darknet is Installed:**
1. wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights  
2. wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg  
3. ./darknet detector demo cfg/coco.data yolov4-tiny.cfg yolov4-tiny.weights -c 0
---
## 2.3 Install Smach Viewer

1. sudo apt-get install python-gi-cairo
2. cd ~/ai_class_ws/src/
3. git clone https://github.com/ros-visualization/executive_smach_visualization.git
4. cd ~/ai_class_ws 
5. catkin_make
6. source ~/.bashrc

&emsp; **Check if Smach Viewer is Installed:**
1. roscore
2. *In a new terminal:*  
	* python ~/ai_class_ws/src/state_machine/src/perfectCase_smach.py  
3. *In a new terminal:*  
	* rosrun smach_viewer smach_viewer.py

---
## 2.4 Install GUI package
1. install dependencies

2. 


---
## 2.5 Run and View all Nodes
1. cd ~/ai_class_ws/src/launchers/
2. chmod +x ../computer_vision/src/cv_node.py ../graphical_user_interface/src/gui_node.py ../guidance_navigation_control/src/gnc_node.py ../sensing_and_actuation/src/sensorActuator_node.py ../state_machine/src/smach_node.py
3. *Choose an Option to Run Nodes:*
	* *Option 1: (Multiple Terminals)*
		* bash node_launcher.bash   
	* *Option 2: (Single Terminal)*
		* roslaunch node_launcher.launch
4. *In a new terminal:*  
	* rosrun rqt_graph rqt_graph   


*The output should look like the following:*

![RQT_Graph](rqt_graph.png)

---
# Topic 3: Training of Neural Network
## 3.1 Image labeling
1. prepare the images from videos
2. labeling tool


## 3.2 Image training
add

