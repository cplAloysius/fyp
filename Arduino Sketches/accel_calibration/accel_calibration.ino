#include <Adafruit_Sensor_Calibration.h>
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc

sensors_event_t mag_event, gyro_event, accel_event;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  
  Wire.setClock(400000); // 400KHz

  Serial.println("turn sensors slowly and steadily in all directions until done");
  delay(5000);
  Serial.println("Starting...");

  for (int i=0; i<350; i++) {
    accelerometer->getEvent(&accel_event);
    magnetometer->getEvent(&mag_event);
    Serial.println(String(accel_event.acceleration.x) + ", " + String(accel_event.acceleration.y) + ", " + 
    String(accel_event.acceleration.z) + ", " + String(mag_event.magnetic.x) + ", " + String(mag_event.magnetic.y) + ", " + 
    String(mag_event.magnetic.z));
    delay(200);
  }
  Serial.println("done collecting");
}

void loop() {
  // put your main code here, to run repeatedly:

}
