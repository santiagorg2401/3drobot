
 
/*
 * 3drobot firmware.
 * 
 */
 
#define F_CPU 12000000
#define pi 3.1415926535897932384626433832795

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <AccelStepper.h>

#define X_STEP_PIN         A0
#define X_DIR_PIN          A1
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         A6
#define Y_DIR_PIN          A7
#define Y_ENABLE_PIN       A2
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       A8
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

// Define stepper motors connection pins (Type:driver, STEP, DIR).
AccelStepper LeftFrontWheel(1, E_STEP_PIN, E_DIR_PIN);  // Stepper1
AccelStepper LeftBackWheel(1, Z_STEP_PIN, Z_DIR_PIN);   // Stepper2
AccelStepper RightBackWheel(1, Y_STEP_PIN, Y_DIR_PIN);  // Stepper3
AccelStepper RightFrontWheel(1, X_STEP_PIN, X_DIR_PIN); // Stepper4
//AccelStepper Zaxis(1, Z_STEP_PIN, Z_DIR_PIN);                           // Stepper5. 

// Global variables.
int wheelSpeed = 0, contseg = 0, cont = 1, load = 0;
int r = 42;                                                                 // Wheel radius.r
int thermistorPin = A13;

float vX = 0, vY = 0;                                                       // Linear speed [m/s].
float vA = 0;                                                               // Angular speed [rad/s].
float stepDelay = 0.005;                                                     // Stepper motor step delay [seconds].
float cmdExtTemp = 0, extTemp = 0;                                          // Command extruder temperature and actual value [°C].

float a1 = pi/4, a2 = 3*pi/4, a3 = 5*pi/4, a4 = 7*pi/4, w = pi, R = 181.07; // Kinematics parameters.
float v1 = 0, v2 = 0, v3 = 0, v4 = 0;                                       //Wheel angular speed [rad/s].
float vs1 = 0, vs2 = 0, vs3 = 0, vs4 = 0;                                   //Wheel angular speed [step/s].

unsigned long timeNow = 0;
int period = stepDelay*1000;

// Set up ROS node.
ros::NodeHandle  nh;

// Callback functions.
void velocityCB(const geometry_msgs::Twist& vel_msg){
  vX = vel_msg.linear.x*1000;
  vY = vel_msg.linear.y*1000;
}

void temperatureCB(const std_msgs::Float32& temp_msg){
  cmdExtTemp = temp_msg.data;
}

// Set up subscibers.
ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", &velocityCB);
ros::Subscriber<std_msgs::Float32> temp_sub("/cmd_extTemp", &temperatureCB);

// Set up publishers.
std_msgs::Float32 extTempMsg;
ros::Publisher extTempPub("/extTemp", &extTempMsg);

void setup(){
  // Set up baud rate.
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);

  // Pin configuration.
  
  pinMode(X_STEP_PIN  , OUTPUT);
  pinMode(X_DIR_PIN    , OUTPUT);
  pinMode(X_ENABLE_PIN    , OUTPUT);

  pinMode(Y_STEP_PIN  , OUTPUT);
  pinMode(Y_DIR_PIN    , OUTPUT);
  pinMode(Y_ENABLE_PIN    , OUTPUT);

  pinMode(Z_STEP_PIN  , OUTPUT);
  pinMode(Z_DIR_PIN    , OUTPUT);
  pinMode(Z_ENABLE_PIN    , OUTPUT);

  pinMode(E_STEP_PIN  , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN    , OUTPUT);

  digitalWrite(X_ENABLE_PIN    , LOW);
  digitalWrite(Y_ENABLE_PIN    , LOW);
  digitalWrite(Z_ENABLE_PIN    , LOW);
  digitalWrite(E_ENABLE_PIN    , LOW);

  // Initialize ROS node.
  nh.initNode();

  // Advertise publishers.
  nh.advertise(extTempPub);

  // Set maximum speeds per motor.
  LeftFrontWheel.setMaxSpeed(2000);
  LeftBackWheel.setMaxSpeed(2000);
  RightBackWheel.setMaxSpeed(2000);
  RightFrontWheel.setMaxSpeed(2000);
}

void loop(){
  // Subscribe from /cmd_vel and /cmd_extTemp.
  nh.subscribe(vel_sub);
  nh.subscribe(temp_sub);

  // Get extruder temperature and publish it in /extTemp.
  extTemp = analogRead(thermistorPin);
  extTempMsg.data = extTemp;
  extTempPub.publish( &extTempMsg );

  // Move the robot..
  calculateSpeeds();
  timeNow = millis();
  
  while (millis() < timeNow + period){
    setExSpeeds();
  }
  
  nh.spinOnce();
}

void calculateSpeeds() {
  // Calculate angular speed per wheel in rad/s.
  v1=(-sin(w+a1)*vX+cos(w+a1)*vY+R*vA)/r;
  v2=(-sin(w+a2)*vX+cos(w+a2)*vY+R*vA)/r;
  v3=(-sin(w+a3)*vX+cos(w+a3)*vY+R*vA)/r;
  v4=(-sin(w+a4)*vX+cos(w+a4)*vY+R*vA)/r;

  // Obtain angular speed in step/s.
  vs1= (v1*180)/1.8;
  vs2= (v2*180)/1.8;
  vs3= (v3*180)/1.8;
  vs4= (v4*180)/1.8;
  
  if(abs(vs1)>500|abs(vs2)>500|abs(vs3)>500|abs(vs4)>500){
    vs1= 0;
    vs2= 0;
    vs3= 0;
    vs4= 0;
  }
 }
 
void setExSpeeds() {
  LeftFrontWheel.setSpeed(vs1);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.setSpeed(vs2);
  LeftBackWheel.runSpeed();
  RightBackWheel.setSpeed(vs3);
  RightBackWheel.runSpeed();
  RightFrontWheel.setSpeed(vs4);
  RightFrontWheel.runSpeed();
}

void stopMoving() {
  LeftFrontWheel.setSpeed(0);
  LeftFrontWheel.runSpeed();
  LeftBackWheel.setSpeed(0);
  LeftBackWheel.runSpeed();
  RightBackWheel.setSpeed(0);
  RightBackWheel.runSpeed();
  RightFrontWheel.setSpeed(0);
  RightFrontWheel.runSpeed();
}