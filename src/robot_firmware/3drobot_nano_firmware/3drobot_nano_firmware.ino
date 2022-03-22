#include <ros.h>
#include <thermistor.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

int thermistorPin = A0;
int weightPin = A1;
int powerPin = 2;

bool powerStage = true;
unsigned int cmdExtTemp = 0, extTemp = 0;
unsigned int weight = 0;

// Set up ROS node.
ros::NodeHandle  nh2;

// Set up thermistor.
thermistor therm1(thermistorPin, 7);

void temperatureCB(const std_msgs::Int16& temp_msg) {
  cmdExtTemp = temp_msg.data;
}

void powerStageCB(const std_msgs::Bool& powerStageMsg) {
  powerStage = powerStageMsg.data;
}

// Set up subscibers.
ros::Subscriber<std_msgs::Int16> temp_sub("/cmd_extTemp", &temperatureCB);
ros::Subscriber<std_msgs::Bool> power_sub("/powerStage", &powerStageCB);

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
  nh2.advertise(extTempPub);
  nh2.advertise(weightPub);

  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH);
}

void loop() {
  nh2.subscribe(temp_sub);
  nh2.subscribe(power_sub);

  setPowerStatus();

  getTemperature();
  getWeight();

  nh2.spinOnce();
}

void setPowerStatus(){
  if (powerStage == false){
    digitalWrite(powerPin, LOW);
  } else{
    digitalWrite(powerPin, HIGH);
  }
}

void getTemperature() {
  // Get extruder temperature and publish it in /extTemp.
  extTemp = therm1.analog2temp();

  extTempMsg.data = extTemp;
  extTempPub.publish( &extTempMsg );
}

void getWeight() {
  // Get extruder temperature and publish it in /extTemp.
  weight = analogRead(weightPin);

  weightMsg.data = weight;
  weightPub.publish( &weightMsg );
}
