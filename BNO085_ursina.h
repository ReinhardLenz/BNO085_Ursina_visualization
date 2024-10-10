// Content from BNO085_ursina.h
#ifndef Compass_H
#define Compass_H
#define BNO085_SAMPLERATE_DELAY_MS (100)
#include <math.h>
#include <Wire.h>
#include <KalmanFilter.h>
#include <Adafruit_BNO08x.h>
#define BNO08X_ADDR 0x4B // Make sure this matches your device
#define RAD_TO_DEG 57.295 // Define RAD_TO_DEG
struct euler_t {
  float yaw;
  float pitch;
  float roll;
};


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
float getNorthDirection(float yaw);

class Compass {
  public:
    Compass() {};
    float getHeading(Adafruit_BNO08x* bno08x, sh2_SensorValue_t* sensorValue); 
    static long getReportInterval() { return reportIntervalUs; }
  private:
    bool activeCalibration_ = false;
    long calibrationResetInterval_ = 1000L * 60L * 60L; // 1 Hour
    long lastCalibrationTime_ = 0L;
    float yaw, pitch, roll;
    float accuracy;
    euler_t ypr;
    int acc_status;
    static const long reportIntervalUs = 100000L;
    KalmanFilter magXFilter_{15, 15, 0, 0.05};
    KalmanFilter magYFilter_{15, 15, 0, 0.05};
    KalmanFilter magZFilter_{15, 15, 0, 0.05};
};
float Compass::getHeading(Adafruit_BNO08x* bno08x, sh2_SensorValue_t* sensorValue) {
  if (bno08x->wasReset()) {
    bno08x->enableReport(SH2_ROTATION_VECTOR, reportIntervalUs);
  }
  if (bno08x->getSensorEvent(sensorValue)) {
    acc_status = sensorValue->status;
    quaternionToEulerRV(&sensorValue->un.rotationVector, &ypr, true); // degrees
    yaw = ypr.yaw;
    pitch = ypr.pitch;
    roll = ypr.roll;
    accuracy = sensorValue->un.rotationVector.accuracy;
    Serial.print("X: "); Serial.print(yaw);
    Serial.print(" \tY: "); Serial.print(pitch);
    Serial.print(" \tZ: "); Serial.print(roll);
    Serial.println("");
  }
  float currentHeading = getNorthDirection(ypr.yaw);
  delay(BNO085_SAMPLERATE_DELAY_MS);
  //Serial.println(currentHeading);
  return currentHeading;
}
float getNorthDirection(float yaw) {
  // Implement this function as needed
  return yaw;
}


#endif
