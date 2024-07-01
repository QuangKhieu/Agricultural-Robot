#include <Wire.h>
#include <Adafruit_10DOF.h>

Adafruit_10DOF sensors;

void setup() {
  Serial.begin(9600);
  sensors.begin();
}

void loop() {
  sensors_event_t event;

  // Read data from the HMC5883L sensor
  sensors.getEvent(&event, Adafruit_10DOF::TYPE_HMC5883L);

  // Access the magnetic field data
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  if (heading < 0) {
    heading += 2 * PI;
  }
  heading = heading * 180 / PI;

  // Print the heading
  Serial.print("Heading: ");
  Serial.println(heading);

  delay(500);
}
