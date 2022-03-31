#include <HX711.h>

#include <ros.h>
#include <thermistor.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

int thermistorPin = A2;
const int DOUT=A1;
const int CLK=A0;
int powerPinPlatform = 2;
int powerPinArm = 4;

int powerStage = 0;
unsigned int cmdExtTemp = 0, extTemp = 0;
unsigned int weight = 0;

// Set up ROS node.
ros::NodeHandle  nh2;

// Set up thermistor.
thermistor therm1(thermistorPin, 7);

HX711 weightSensor;

void temperatureCB(const std_msgs::Int16& temp_msg) {
  cmdExtTemp = temp_msg.data;
}

void powerStageCB(const std_msgs::Float32& powerStageMsg) {
  powerStage = powerStageMsg.data;
  
  if (powerStage == 1.0){
    digitalWrite(powerPinPlatform, LOW);
  } 

  else if(powerStage == 2.0){
    digitalWrite(powerPinArm, HIGH);
    }
    
  else if(powerStage == 3.0){
    digitalWrite(powerPinPlatform, HIGH);
  }
  
  else if(powerStage == 4.0){
    digitalWrite(powerPinArm, LOW);
  }
}

// Set up subscibers.
ros::Subscriber<std_msgs::Int16> temp_sub("/cmd_extTemp", &temperatureCB);
ros::Subscriber<std_msgs::Float32> power_sub("/powerStage", &powerStageCB);

// Set up publishers.

std_msgs::Int16 extTempMsg;
ros::Publisher extTempPub("/extTemp", &extTempMsg);

std_msgs::Int16 weightMsg;
ros::Publisher weightPub("/weight", &weightMsg);

void setup() {
  // Set up baud rate.
  Serial.begin(115200);
  nh2.getHardware()->setBaud(115200);

  // Initialize ROS node.
  nh2.initNode();

  // Advertise publishers.
//  nh2.advertise(extTempPub);
//  nh2.advertise(weightPub);
//
//  weightSensor.begin(DOUT,CLK);
//  weightSensor.set_scale();
//  weightSensor.tare(20);

  pinMode(powerPinPlatform, OUTPUT);
  pinMode(powerPinArm, OUTPUT);

  digitalWrite(powerPinPlatform, HIGH);
  digitalWrite(powerPinArm, LOW);

  
}

void loop() {
//  nh2.subscribe(temp_sub);
  nh2.subscribe(power_sub);


//  getTemperature();
//  getWeight();

  nh2.spinOnce();
  
}


void getTemperature() {
  // Get extruder temperature and publish it in /extTemp.
  extTemp = therm1.analog2temp();

  extTempMsg.data = extTemp;
  extTempPub.publish( &extTempMsg );
}

void getWeight() {
  // Get extruder temperature and publish it in /extTemp.
  weight = weightSensor.get_value(10);

  weightMsg.data = weight;
  weightPub.publish( &weightMsg );
}
