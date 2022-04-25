#include <HX711.h>
#include <thermistor.h>

int thermistorPin = A2;
const int DOUT = A1;
const int CLK = A0;
int powerPinPlatform = 2;
int powerPinArm = 4;

int powerStage = 0;
unsigned int cmdExtTemp = 0, extTemp = 0;
float weight = 0;

// Set up thermistor.
thermistor therm1(thermistorPin, 7);

HX711 weightSensor;

void setup() {
  // Set up baud rate.
  Serial.begin(9600);

  weightSensor.begin(DOUT, CLK);
  weightSensor.set_scale(439430.25);
  weightSensor.tare(20);

}

void loop() {
  getTemperature();
  getWeight();
  String json1 = "{\"temperature\" : " + String(extTemp) + ", \"weight\" : " + String(weight) + "}";
  Serial.println(json1);
  delay(100);
}


void getTemperature() {
  // Get extruder temperature and publish it in /extTemp.
  extTemp = therm1.analog2temp();
}

void getWeight() {
  // Get extruder temperature and publish it in /extTemp.
  weight = weightSensor.get_units(20);
}
