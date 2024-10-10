#include "BNO085_ursina.h"
// Declare objects and variables
Adafruit_BNO08x bno08x = Adafruit_BNO08x();
sh2_SensorValue_t sensorValue;
Compass compass;
unsigned long millisOld;
float lastSerialHeading;
unsigned long lastSerialRecv;
const unsigned long serialKeepDurationMillis = 5000; // Adjust this value as needed
void setup() {
  Serial.begin(9600); // Change this to 230400 to match the working example
  while (!Serial) delay(10);
  Serial.println("Serial started. Try and initialise BNO08x.");

  // Initialize the I2C sensor with the specific address
  if (!bno08x.begin_I2C(BNO08X_ADDR, &Wire)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10); // Placeholder for failure handling
    }
  }

  Serial.println("BNO08x Found!");
  // Set desired reports for the sensor
  setReports(compass.getReportInterval());
  millisOld = millis();
}
void loop() {
  float heading = compass.getHeading(&bno08x, &sensorValue);
  if (Serial.available() > 0) {
    String serialHeading = Serial.readString();
    lastSerialHeading = serialHeading.toFloat();
    lastSerialRecv = millis();
  }

  if (millis() - lastSerialRecv < serialKeepDurationMillis) {
    heading = lastSerialHeading;
  }
}
void setReports(long reportIntervalUs) {
  // Set the desired reports for the sensor
  bno08x.enableReport(SH2_ROTATION_VECTOR, reportIntervalUs);
}
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) {
  // Normalize quaternion before conversion.
  float qlength = sqrt(sq(qr) + sq(qi) + sq(qj) + sq(qk));
  qr /= qlength;
  qi /= qlength;
  qj /= qlength;
  qk /= qlength;
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);
  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqr - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqr + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqr - sqj + sqk + sqr));
  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}
