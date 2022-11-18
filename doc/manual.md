# Manual


---
## 1. mechanical system

### 1.1 overall design


---
### 1.2 mechanical analysis 


---
## 2. electrical system design

### 2.1 overall circuit diagram  

A typical electrical system on a robot should include the following subsystems
1. main computer: coordinate the mission of the robot
2. microcontrollers: help the main computer get some tasks done
3. sensors: understand the enviornment and itself
4. actuators: interact with environment
5. power management: supply reliable power to all units
6. battery: main hub that stores and supply power

---
### 2.2 main computer
This robot use Jetson Nano 2Gb as its main computer. The introdcution of this board can be found here: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/education-projects/  

---
### 2.3 microcontrollers
This robot uses an arduino microcontrollers MEGA (fully compatable with other similar micontrolers). You can find more infomation about this microcontroller here: https://store-usa.arduino.cc/products/arduino-mega-2560-rev3?selectedStore=us

---
### 2.4 power management
Since a robotic system uses many electrical components, which may require different power supplies. From the basic knowlege in electrical circuits, we know the ```power = voltage * current```. Hence, we only need to ensure the the power outlet prepared for an electrical compoent has the right voltage and maximum current (or power). In practice, a robot only has only one battery (bank) with a specific voltage, to prepare multiple power outlets for other electrical compnents, there are only two types of operations, either increase or decrease the voltage.  

(1) To decrease the DC voltage, two type of devices are available - linear voltage regulator and buck converter. 

Linear voltage regulator: use a transistor controlled by a negative feedback circuit to produce a specified output voltage. The output remains stable despite variations in load current and input voltage. They have advantages of ease of use, low output noise, low cost, but have low efficiency and generate a lot of heat in opertaion. Need to consider thermal designs when they are used.   

Buck converter (switch regulators): use pulse width modulation to switch the circuit on and off at a high efficiency to obain a reduced voltage. The output has ripple effecit, but the efficiency is very high, often >90%. 
This robot uses a buck converter chip XL4015. You can check its datasheet for details on the features, transient characteristics, typical applications, and more. 

(2) To increase the DC voltage, boost converters are used. Booster converters belong to switch regulators. Different from a buck converter, boost converters first convert DC to high frequence AC, then use a transformer to boost its voltage, and final use a filter to smooth the output. 

A more comprehensive comparison of linear and switching regulators can be found here: https://www.digikey.com/en/articles/understanding-the-advantages-and-disadvantages-of-linear-regulators

| Item          | Linear        | Switching  |
| ------------- |:-------------:| -----:|
| Function      | Only steps down (buck) so input voltage must be greater than output voltage | Step up (boost), step down (buck), inverts |
| Efficiency    | Low to medium, but actual battery life depends on load current and battery voltage over time. Efficiency is high if difference between input and output voltages is small |  High, except at very low load currents (μA), where switch-mode quiescent current (IQ) is usually higher |
| Waste heat    | High, if average load and/or input to output voltage difference are high | Low, as components usually run cool for power levels below 10 W |
| Complexity    | Low, usually requiring only the regulator and low value bypass capacitors | Medium to high, usually requiring inductor, diode, and filter caps in addition to the IC; for high-power circuits, external FETs are needed |
| Size          | Small to medium in portable designs, but may be larger if heatsinking is needed | Larger than linear at low power, but smaller at power levels for which linear requires a heat sink |
| Total cost	| Low | Medium to high, largely due to external components |
| Ripple/Noise	| Low; no ripple, low noise, better noise rejection	| Medium to high, due to ripple at switching rate |

---
### 2.5 sensors

With a focus on only the fundamentals, this robot 
only has some of the most basic but important sensors - depth sensor, and inertial measurement unit (IMU). To be accurate, IMU integrates multiple sensors, including a magnetometer, gyroscopes, and accelerometers. More information on the IMU can be found here: https://www.adafruit.com/product/2472.



---
## 3. software 

### 3.1 overall design

The main system archictecture of the robot can be divided into three main categories: sense, think, and act. 

(1) With the sensing functions, the robot is able to gather information about its surroundings and its current location relative to the objects around it. 

(2) During the thinking phase, the robot inputs its gathered information into its navigation and path planning algorithms and decides on the next course of action through setpoints.

(3) During the action phase, the robot takes the setpoints and its current location and uses a control system to navigate to the desired location.

![SenseThinkAct](/fig/SenseThinkAct.png)

Fig. 1 - Sense Think Act Paradigm [1]


To go more in-depth, the system archictecture tends to be seen as a multi-layered system which allows for code reusage, ease of testing, and increase scalability:

A host layer, or user interface layer, is generally used for the user to observe all of the sensor information and actuator status, as well as information on the robot's health. 

Underneath this host layer is the embedded hardware, which utilizes an algorithm layer (path planning, mapping, target recognition, etc.), a platform layer (navigation, filtering, etc.), and a driver layer (sensor data acquisiton, actuator movements, control system, etc.).

![SystemArchitecture](/fig/System_Architecture.png)

Fig. 2 - Overall Software Design

---
### 3.2 microcontroller, sensors, and actuators

You can follow the above two links to learn how to read sensor data from an arduino microcontroller. 

The depth sensor is from blue robotics. To read the depth data is very straightforward, check this link for details. https://github.com/bluerobotics/BlueRobotics_MS5837_Library/blob/master/examples/MS5837_Example/MS5837_Example.ino

The inertial measurement unit is from adafruit. To read sensor informaiton from the IMU, you can refere to this page for detais: https://www.adafruit.com/product/2472. 

---
### 3.3 robot operating system (ROS)

The Robot Operating System, or ROS, is an open-source software framework with an extensive number of libraries and tools for use in robotics applications. 


A ROS node is an executable file, in our case a python file, which will run simultaneously with other nodes while being capable of communicating with them.
Every ROS node is either a publisher or subscriber. 
- Publishers print out specified data to a topic.
- Subscribers listen to the topic and collect the specific data they require.


Multiple nodes can publish to one topic, and multiple nodes may subscribe to the same topic. On top of this, multiple topics can also be made to organize who is getting what data.


![SystemArchitecture](/fig/Nodes-TopicandService.gif)

Fig. 3 - ROS Node Communication [2]

For our robot, five main nodes communicate with each other over four topics. Each node can be seen in the following list or in Figure 4 below with the topics it publishes and subscribes to:
1. **Computer Vision**
    * Publisher: *Target*
    * Subscriber: *n/a*
2. **State Machine**
    * Publisher: *Task_DesiredAction*
    * Subscriber: *Target, ControlCommand, SensorInfo_ActuatorStatus*
3. **Guidance, Navigation, and Control**
    * Publisher: *ControlCommand*
    * Subscriber: *Task_DesiredAction, SensorInfo_ActuatorStatus*
4. **Sensors and Actuators**
    * Publisher: *SensorInfo_ActuatorStatus*
    * Subscriber: *ControlCommand*
5. **Graphical User Interface**
    * Publisher: *n/a*
    * Subscriber: *Task_DesiredAction, SensorInfo_ActuatorStatus*

![RQT_Graph](/fig/rqt_graph.png)

Fig. 4 - Robot RQT Graph

---
#### 3.4 Jetson Nano, microcontroller


---
### 3.5 mission planner


---
### 3.6 control system

The control system used in the robot utilizes multiple PIDs for the x, y, z directions, and yaw, pitch, roll orientations (see Fig 6 below). PID is an acryonym for a Proportional (P), Integral (I), Derivative (D) controller. 

![PID_Equation](/fig/PID_Equation.png)

Fig. 5 - PID Controller Equation [3]

When the robot attempts to reach a new setpoint, the difference between the setpoint and its current position is calculated and is known as its error. The PID controller takes this error as an input and outputs a PWM speed for the thrusters so the robot can approach the setpoint.

![RobotDirectionsAndOrientations](/fig/robot_xyz_ypr.png)

Fig. 6 - Robot Directions and Orientations

Before the PID controller can take the error and output the correct PWM, it must first be tuned. Tuning involves the changing of the Kp, Ki, and Kd values in the equation to achieve a close to ideal, stable system with a fast response time. 

Although many tuning methods have been created, the figure below shows the process of manually tuning the controller.

![PID_Tuning](/fig/PID_Tuning_Animated.gif)

Fig. 7 - Manual PID Tuning [4]





---
## 4. references
[1] “Improve Efficiency with Flexible Automation.” CrossCo, 3 Dec. 2020, https://www.crossco.com/resources/technical/fixed-versus-flexible-automation/. 

[2] “Understanding Nodes.” ROS 2 Documentation: Rolling, https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html. 

[3] “What Is a PID Controller and How It Works?” PLCynergy, 14 Oct. 2022, https://plcynergy.com/pid-controller/. 

[4] “PID Compensation Animated.” Wikimedia Commons, https://commons.wikimedia.org/wiki/File:PID_Compensation_Animated.gif. 

