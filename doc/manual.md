# Manual


---
## 1. mechanical system

### 1.1 overall design


---
### 1.2 mechanical analysis 


---
## 2. electrical system design

### 2.1 overall circuit diagram  

A tyical electrical system on a robot should include the following subsystems
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
only has some of the most basic but important sensors - depth sensor, and inertial measurement unit(IMU). To be accurate, IMU integrats multiple sensors, including a magnetometer, gyroscopes, and accelerometers. More information on the IMU can be found here: https://www.adafruit.com/product/2472.



## 3. software 

### 3.1 overall design
(add a fig here to show the overall design)
Fig. Overall sotware desgin

---
### 3.2 microcontroller, sensors, and actuators

You can follow the above two links to learn how to read sensor data from an arduino microcontroller. 

The depth sensor is from blue robotics. To read the depth data is very straightforward, check this link for details. https://github.com/bluerobotics/BlueRobotics_MS5837_Library/blob/master/examples/MS5837_Example/MS5837_Example.ino

The inertial measurement unit is from adafruit. To read sensor informaiton from the IMU, you can refere to this page for detais: https://www.adafruit.com/product/2472. 

---
### 3.3 robot operating system (ROS)


---
#### 3.4 Jetson Nano, microcontroller


---
### 3.5 mission planner







