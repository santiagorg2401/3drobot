/*
 * 3drobot firmware.
 * 
 */

#define F_CPU 12000000
#define pi 3.1415926535897932384626433832795
#include <AccelStepper.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
 
// Voor de Arduino Mega + shield Ramps 1.4
#define MOTOR_X_STEP_PIN 54
#define MOTOR_X_DIR_PIN 55
#define MOTOR_X_ENABLE_PIN 38
 
#define MOTOR_Y_STEP_PIN 60
#define MOTOR_Y_DIR_PIN 61
#define MOTOR_Y_ENABLE_PIN 56
 
#define MOTOR_Z_STEP_PIN 46
#define MOTOR_Z_DIR_PIN 48
#define MOTOR_Z_ENABLE_PIN 62

#define X_ENDSTOP 3   /* X axis endstop input pin */
#define Y_ENDSTOP 14  /* Y axis endstop input pin */
#define Z_ENDSTOP 18  /* Z axis endstop input pin */

#define E_STEP_PIN 26
#define E_DIR_PIN 28
#define E_ENABLE_PIN 24


AccelStepper motorX(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN); 
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN); 
AccelStepper motorZ(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN); 

int speedX=100;
int speedY=100;
int speedZ=100;
int targetX=0;
int targetY=0;
int targetZ=0;

String str;

unsigned char XendStop=0;
unsigned char YendStop=0;
unsigned char ZendStop=0;
unsigned char calcomp=0;

// Set up ROS node.
ros::NodeHandle  nh;

void xCB(const std_msgs::Float64& x_msg){
  targetX=x_msg.data/1.8;

  if(x_msg.data==999){
      CalibrationX();
    }
  else{
    goToTargetXY(); 
  }
}

void yCB(const std_msgs::Float64& y_msg){
  targetY=y_msg.data/1.8;

  if(y_msg.data==999){
      CalibrationY();
    }
  else{
    goToTargetXY(); 
  }
}

void zCB(const std_msgs::Float32& z_msg){
  targetZ=z_msg.data*200/8;  // 8 milimetros de avance por vuelta, y como el paso del motor es de 1.8°.
                      // 8 mm de avance es igual a 200 pasos (360°/1.8). Si el paso del motor cambia
                      // será necesario dividir 360 entre el nuevo paso y reemplazar el 200 por este valor
  if(z_msg.data==999){
      CalibrationZ();
    }
  else{
    goToTargetZ(); 
  }
}


// Set up subscibers.
ros::Subscriber<std_msgs::Float64> x_sub("/robot1/joint1_position_controller/command", &xCB);
ros::Subscriber<std_msgs::Float64> y_sub("/robot1/joint2_position_controller/command", &yCB);
ros::Subscriber<std_msgs::Float32> z_sub("/cmd_zAxisPos", &zCB);

// Set up publishers.
std_msgs::Float32 TargetReachedMsg;
ros::Publisher TargetReachedPub("/TargetReached", &TargetReachedMsg);



void setup() {

  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  
  // Initialize ROS node.
  nh.initNode();
  
  // Advertise publishers.
  nh.advertise(TargetReachedPub);

  pinMode(MOTOR_X_STEP_PIN  , OUTPUT);
  pinMode(MOTOR_X_DIR_PIN    , OUTPUT);
  pinMode(MOTOR_X_ENABLE_PIN    , OUTPUT);

  pinMode(MOTOR_Y_STEP_PIN  , OUTPUT);
  pinMode(MOTOR_Y_DIR_PIN    , OUTPUT);
  pinMode(MOTOR_Y_ENABLE_PIN    , OUTPUT);

  pinMode(MOTOR_Z_STEP_PIN  , OUTPUT);
  pinMode(MOTOR_Z_DIR_PIN    , OUTPUT);
  pinMode(MOTOR_Z_ENABLE_PIN    , OUTPUT);
  

  digitalWrite(MOTOR_X_ENABLE_PIN    , LOW);
  digitalWrite(MOTOR_Y_ENABLE_PIN    , LOW);
  digitalWrite(MOTOR_Z_ENABLE_PIN    , LOW);

  
  motorX.setMaxSpeed(500);
  motorX.setSpeed(speedX);
  motorX.setCurrentPosition(0);
  
  motorY.setMaxSpeed(500);
  motorY.setSpeed(speedY);
  motorZ.setCurrentPosition(0);
 
  motorZ.setMaxSpeed(500);
  motorZ.setSpeed(speedZ);
  motorZ.setCurrentPosition(0);

  pinMode(X_ENDSTOP, INPUT_PULLUP);
  pinMode(Y_ENDSTOP, INPUT_PULLUP);
  pinMode(Z_ENDSTOP, INPUT_PULLUP);
}
 
void loop() {

  nh.subscribe(x_sub); 
  nh.subscribe(y_sub); 
  nh.subscribe(z_sub);   
  nh.spinOnce(); 
      
}

void CalibrationX(){
  
    XendStop=digitalRead(X_ENDSTOP);
    while(XendStop==1){ 
      XendStop=digitalRead(X_ENDSTOP);  
      motorX.setSpeed(speedX);
      motorX.runSpeed(); 
      motorX.setCurrentPosition(0);
      TargetReachedMsg.data = 0;
    }
        
    TargetReachedMsg.data = 1;
    TargetReachedPub.publish( &TargetReachedMsg );
    TargetReachedMsg.data = 0;
    TargetReachedPub.publish( &TargetReachedMsg );
}

void CalibrationY(){
  
    YendStop=digitalRead(Y_ENDSTOP);
    while(YendStop==1){ 
      YendStop=digitalRead(Y_ENDSTOP);  
      motorY.setSpeed(speedY);
      motorY.runSpeed(); 
      motorY.setCurrentPosition(0);
      TargetReachedMsg.data = 0;
    }
        
    TargetReachedMsg.data = 1;
    TargetReachedPub.publish( &TargetReachedMsg );
    TargetReachedMsg.data = 0;
    TargetReachedPub.publish( &TargetReachedMsg );
}
  
void CalibrationZ(){
  
  ZendStop=digitalRead(Z_ENDSTOP);
    while(ZendStop==1){ 
      ZendStop=digitalRead(Z_ENDSTOP);  
      motorZ.setSpeed(speedZ);
      motorZ.runSpeed(); 
      motorZ.setCurrentPosition(0);
      TargetReachedMsg.data = 0;
    }
    
    TargetReachedMsg.data = 1;
    TargetReachedPub.publish( &TargetReachedMsg );
    TargetReachedMsg.data = 0;
    TargetReachedPub.publish( &TargetReachedMsg );
}

void goToTargetXY(){
  
  while(motorX.currentPosition()!=targetX || motorY.currentPosition()!=targetY){
    
    if (motorX.currentPosition()==targetX){
      motorX.setSpeed(0);
      motorX.runSpeed(); 
    }
    else if (motorX.currentPosition()<targetX){
      motorX.setSpeed(speedX);
      motorX.runSpeed(); 
    }
    else if(motorX.currentPosition()>targetX){
      motorX.setSpeed(-speedX);
      motorX.runSpeed();
    }

    if (motorY.currentPosition()==targetY){
      motorY.setSpeed(0);
      motorY.runSpeed(); 
    }
    else if (motorY.currentPosition()<targetY){
      motorY.setSpeed(speedY);
      motorY.runSpeed(); 
    }
    else if(motorY.currentPosition()>targetY){
      motorY.setSpeed(-speedY);
      motorY.runSpeed();
    }
  } 

  TargetReachedMsg.data = 1;
  TargetReachedPub.publish( &TargetReachedMsg );
  TargetReachedMsg.data = 0;
  TargetReachedPub.publish( &TargetReachedMsg );
  
}

void goToTargetZ(){
  
  while(motorZ.currentPosition()!=targetZ){
    if (motorZ.currentPosition()<targetZ){
      motorZ.setSpeed(speedZ);
      motorZ.runSpeed(); 
    }
    if(motorZ.currentPosition()>targetZ){
      motorZ.setSpeed(-speedZ);
      motorZ.runSpeed();
    }
   }
  motorZ.setSpeed(0);
  motorZ.runSpeed();
  TargetReachedMsg.data = 1;
  TargetReachedPub.publish( &TargetReachedMsg );
  TargetReachedMsg.data = 0;
  TargetReachedPub.publish( &TargetReachedMsg );
  
}
  
