/* Full Serial Input Thruster Testing

   Current Revision 4 26 21:
      Horizontal and Vertical Thrusters React
      Can Change Setpoints from Serial:
          (Type letters in serial to move sub)
      Added roll and pitch to manual input
      Deleted path optimization for roll and pitch
      Added ROS information
      
   Written by: Aren Petrossian
   All values with "//****"  can be personalized
*/


//All librarys and IMU and Barometer setup
#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"
#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
MS5837 sensor;



//Start thrusters as servos and set Pins**
Servo left_thruster;
Servo right_thruster;
Servo front_right_thruster;
Servo front_left_thruster;
Servo back_left_thruster;
Servo back_right_thruster;

double final_left_thruster_value;
double final_right_thruster_value;
double final_front_right_thruster_value;
double final_front_left_thruster_value;
double final_back_left_thruster_value;
double final_back_right_thruster_value;

int left_thruster_pin = 11;                 //****
int right_thruster_pin = 10;                //****
int front_right_thruster_pin = 5;           //****
int front_left_thruster_pin = 6;            //****
int back_left_thruster_pin = 9;             //****
int back_right_thruster_pin = 7;            //****

double H_distance_input, H_distance_output, H_distance_setpoint;
double barometer_input, barometer_output, barometer_setpoint;

double X_angle_input, X_angle_output, X_angle_setpoint;
double Y_angle_input, Y_angle_output, Y_angle_setpoint;
double Z_angle_input, Z_angle_output, Z_angle_setpoint;
double temp_input;
unsigned long lastUnstable = 0;

double H_distance_kP = 1;            //****
double H_distance_kI = 0;            //****
double H_distance_kD = 0;            //****
double barometer_kP = 300;           //****   400 might be too high
double barometer_kI = 0;             //****
double barometer_kD = 0;             //****

double X_angle_kP = 3.5;              //****
double X_angle_kI = 0.0005;           //****
double X_angle_kD = 4;                //****
double Y_angle_kP = 3.5;              //****
double Y_angle_kI = 0.0005;           //****
double Y_angle_kD = 4;                //****
double Z_angle_kP = 3.5;              //****
double Z_angle_kI = 0.0005;           //****
double Z_angle_kD = 4;                //****

PID H_distance_PID  (&H_distance_input, &H_distance_output, &H_distance_setpoint,
                     H_distance_kP, H_distance_kI, H_distance_kD, DIRECT);
PID barometer_PID  (&barometer_input, &barometer_output, &barometer_setpoint,
                    barometer_kP, barometer_kI, barometer_kD, DIRECT);

PID X_angle_PID    (&X_angle_input, &X_angle_output, &X_angle_setpoint,
                     X_angle_kP, X_angle_kI, X_angle_kD, DIRECT);
PID Y_angle_PID    (&Y_angle_input, &Y_angle_output, &Y_angle_setpoint,
                    Y_angle_kP, Y_angle_kI, Y_angle_kD, DIRECT);
PID Z_angle_PID    (&Z_angle_input, &Z_angle_output, &Z_angle_setpoint,
                    Z_angle_kP, Z_angle_kI, Z_angle_kD, DIRECT);


//Set up all ROS Libraries and Pub & Sub
#include <ros.h>
#include <guidance_navigation_control/sensorInfo_actuatorStatus.h>
#include <guidance_navigation_control/controlCommand.h>
ros::NodeHandle nh;
guidance_navigation_control::sensorInfo_actuatorStatus final_message;
ros::Publisher sensorInfo_actuatorStatus("sensorInfo_actuatorStatus", &final_message);

//Calculates PIDs and sends Outputs
void Outputs(const guidance_navigation_control::controlCommand& controls_data) {
  X_angle_setpoint = controls_data.yaw_set;
  Y_angle_setpoint = controls_data.pitch_set;
  Z_angle_setpoint = controls_data.roll_set;
  H_distance_setpoint = controls_data.distance_set;
  //barometer_setpoint = controls_data.depth_set;
  Manual_INPUT();
  Path_Optimization();
  H_distance_PID.Compute();
  barometer_PID.Compute();
  X_angle_PID.Compute();
  Y_angle_PID.Compute();
  Z_angle_PID.Compute();
  Mixer();

  Serial_Print_Thruster_Values();
  left_thruster.writeMicroseconds(final_left_thruster_value);
  right_thruster.writeMicroseconds(final_right_thruster_value);
  front_right_thruster.writeMicroseconds(final_front_right_thruster_value);
  front_left_thruster.writeMicroseconds(final_front_left_thruster_value);
  back_left_thruster.writeMicroseconds(final_back_left_thruster_value);
  back_right_thruster.writeMicroseconds(final_back_right_thruster_value);
}

ros::Subscriber<guidance_navigation_control::controlCommand> sub("controlCommand", Outputs);



//Basic setup steps
void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(sensorInfo_actuatorStatus);
  nh.subscribe(sub);
  InitializeIMU();
  //InitializeBarometer();
  InitializeControls();

  delay(5000);
}



//Plugs IMU angle into PID and moves Thrusters
void loop() {
  IMU_XYZ_angles();
  //Barometer_Reading();
  Publisher_data();
  sensorInfo_actuatorStatus.publish(&final_message);
  nh.spinOnce();
}



//Turns IMU data collecting on
void InitializeIMU() {
  //Wire.begin();       //Can use this if dealing with only 1 sensor
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR");
    while (1);
  }
  bno.setExtCrystalUse(true);
}



//Turns Barometer data collecting on
void InitializeBarometer() {
  //Wire.begin();       //Can use this if dealing with only 1 sensor
  Wire.beginTransmission(0x76);
  while (!sensor.init()) {
    Serial.println("Initilization failed");
    delay(2000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);      //(air ~ 1.23, freshwater ~ 997, seawater ~1029)
}                                   //air is not accurate



//Attaches thrusters and initializes PIDs
void InitializeControls() {
  left_thruster.attach(left_thruster_pin);
  right_thruster.attach(right_thruster_pin);
  front_right_thruster.attach(front_right_thruster_pin);
  front_left_thruster.attach(front_left_thruster_pin);
  back_left_thruster.attach(back_left_thruster_pin);
  back_right_thruster.attach(back_right_thruster_pin);

  H_distance_PID.SetMode(AUTOMATIC);
  barometer_PID.SetMode(AUTOMATIC);
  X_angle_PID.SetMode(AUTOMATIC);
  Y_angle_PID.SetMode(AUTOMATIC);
  Z_angle_PID.SetMode(AUTOMATIC);

  H_distance_PID.SetOutputLimits(-300.00, 300.00);     //assuming max power is +-300
  barometer_PID.SetOutputLimits(-300.00, 300.00);     //assuming max power is +-300
  X_angle_PID.SetOutputLimits(-100.00, 100.00);
  Y_angle_PID.SetOutputLimits(-100.00, 100.00);
  Z_angle_PID.SetOutputLimits(-100.00, 100.00);

  H_distance_input = 0;              //The distance input will always be 0

  H_distance_setpoint = 0;           //****
  barometer_setpoint = 0.15;         //****
  X_angle_setpoint = 0;              //****
  Y_angle_setpoint = 0;              //****
  Z_angle_setpoint = 0;              //****
}



//Gets the X, Y, and Z angles from the IMU
void IMU_XYZ_angles() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  X_angle_input = euler.x();    // our X is yaw
  Y_angle_input = euler.y();    // our Y is pitch
  Z_angle_input = euler.z();    // our Z is roll
  temp_input = bno.getTemp();
//  Serial.print(X_angle_input);
//  Serial.print("----");
//  Serial.print(Y_angle_input);
//  Serial.print("----");
//  Serial.println(Z_angle_input);
}



//Gets the Depth from the Barometer
void Barometer_Reading() {
  sensor.read();
  barometer_input = sensor.depth();

//Don't need any of these for sub to run
/* 
   Serial.print("Depth: ");             //Use Depth in incompressible liquid only
   Serial.print(sensor.depth()); 
   Serial.println(" m");
   
   Serial.print("Pressure: "); 
   Serial.print(sensor.pressure()); 
   Serial.println(" mbar");
   
   Serial.print("Temperature: "); 
   Serial.print(sensor.temperature()); 
   Serial.println(" deg C");
  
   Serial.print("Altitude: ");         //Use Altitude in air only (not very good data)
   Serial.print(sensor.altitude()); 
   Serial.println(" m above mean sea level");
*/
}



//Updates Variables for Publisher 
void Publisher_data() {
  final_message.stabilized = isStabilized();
  final_message.yaw_current = X_angle_input;
  final_message.pitch_current = Y_angle_input;
  final_message.roll_current = Z_angle_input;
  final_message.depth_current = barometer_input;
  final_message.temperature = temp_input;
  final_message.thruster_values[0] = final_left_thruster_value;
  final_message.thruster_values[1] = final_right_thruster_value;
  final_message.thruster_values[2] = final_front_left_thruster_value;
  final_message.thruster_values[3] = final_front_right_thruster_value;
  final_message.thruster_values[4] = final_back_left_thruster_value;
  final_message.thruster_values[5] = final_back_right_thruster_value;
}



//Checks if Robot is Stabilized
bool isStabilized() {
  if ((abs(X_angle_setpoint - X_angle_input) < 5) && (abs(Y_angle_setpoint - Y_angle_input) < 5) &&
      (abs(Z_angle_setpoint - Z_angle_input) < 5) && (abs(barometer_setpoint - barometer_input) < 0.1524) &&
      (H_distance_setpoint == 0)) 
      {
        if ((millis() - lastUnstable) > 3000) {
          return true;
        }
        return false;
      }
  lastUnstable = millis();
  return false;
}



//Checks for User Input
void Manual_INPUT() {
  int Manual_Input = 0;
  if (Serial.available() > 0) {
    Manual_Input = Serial.read();
    Serial.print("Manual_Input = ");
    Serial.println(Manual_Input);
  }
  if (isEqual(Manual_Input)) {
    switch (Manual_Input) {
      case 119: //w       Forwards
        H_distance_setpoint = H_distance_setpoint + 20;
        break;

      case 97:  //a       Yaw Left 10 Degrees
        X_angle_setpoint = X_angle_setpoint - 10;
        break;

      case 115: //s       Backwards
        H_distance_setpoint = H_distance_setpoint - 20;
        break;

      case 100: //d       Yaw Right 10 Degrees
        X_angle_setpoint = X_angle_setpoint + 10;
        break;

      case 32:  //space   Up 0.25 m
        barometer_setpoint = barometer_setpoint - 0.25;
        break;

      case 118: //v       Down 0.25 m
        barometer_setpoint = barometer_setpoint + 0.25;
        break;
        
      case 101:  //e      Roll Left 10 Degrees
        Y_angle_setpoint = Y_angle_setpoint - 10;
        break;

      case 114: //r       Roll Right 10 Degrees
        Y_angle_setpoint = Y_angle_setpoint + 10;
        break;
                
      case 116:  //t      Pitch Up 10 Degrees
        Z_angle_setpoint = Z_angle_setpoint + 10;
        break;

      case 103: //g       Pitch Down 10 Degrees
        Z_angle_setpoint = Z_angle_setpoint - 10;
        break;
    }
  }
}



//Checks if Manual_Input is one of the keys
bool isEqual(int x) {
  int Check[] = {119, 97, 115, 100, 32, 118, 101, 114, 116, 103};       //w,a,s,d,space,v,e,r,t,g

  for (int i = 0; i < 10; i++) {
    if (Check[i] = x) {
      return true;
    }
  }
  return false;
}



//Takes Error and Chooses which Direction to turn
void Path_Optimization() {
  double half_circle_pos = 180.00;
  double half_circle_neg = -180.00;
  double X_angle_error = X_angle_setpoint - X_angle_input;

  if (X_angle_error > half_circle_pos) {
    X_angle_error = X_angle_error - 360;
  }
  else if (X_angle_error < half_circle_neg) {
    X_angle_error = X_angle_error + 360;
  }
  
  X_angle_setpoint = X_angle_error + X_angle_input;
}



//Takes PID outputs and makes pwm for Thrusters
void Mixer() {
  final_left_thruster_value = 1500 + H_distance_output + X_angle_output;
  final_right_thruster_value = 1500 + H_distance_output - X_angle_output;
  final_front_right_thruster_value = 1500 + barometer_output + Y_angle_output + Z_angle_output;
  final_front_left_thruster_value = 1500 + barometer_output + Y_angle_output - Z_angle_output;
  final_back_left_thruster_value = 1500 + barometer_output - Y_angle_output - Z_angle_output;
  final_back_right_thruster_value = 1500 + barometer_output - Y_angle_output + Z_angle_output;
}



//Print All Thrusters Values
void Serial_Print_Thruster_Values() {
  Serial.print("Left: ");
  Serial.print(final_left_thruster_value);
  Serial.print("-------");
  Serial.print("Right: ");
  Serial.print(final_right_thruster_value);
  Serial.print("-------");
  Serial.print("FrontRight: ");
  Serial.print(final_front_right_thruster_value);
  Serial.print("----");
  Serial.print("FrontLeft: ");
  Serial.print(final_front_left_thruster_value);
  Serial.print("----");
  Serial.print("BackLeft: ");
  Serial.print(final_back_left_thruster_value);
  Serial.print("----");
  Serial.print("BackRight: ");
  Serial.print(final_back_right_thruster_value);
  Serial.print("----");
  Serial.println(barometer_input);
}
