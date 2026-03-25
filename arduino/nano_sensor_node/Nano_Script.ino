#include <Arduino_LSM6DSOX.h>
#include "pio_encoder.h"

PioEncoder encoder(15);

long currentPosition;
float deg;
float curPosRad;
float prevPosRad;
float actualVelocity;
float actualRpm;
float prevActualRpm;
float Rpm_filt;
long curTime;
float deltaTimeEnc;
long prevTime;


// IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()

float AccErrorX = 0.01;
float AccErrorY = -0.02;
float AccErrorZ = 0.00;
float GyroErrorX = 0.04;
float GyroErrorY = -0.32;
float GyroErrorZ = -0.20;

// IMU variables

#define LOOP_TIMING 200
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float deltaTime;
float invFreq = (1.0 / LOOP_TIMING) * 1000000.0;
unsigned long current_time, prev_time;

// IMU filter parameters
float Gyro_filter = .97;
float Accel_filter = .97;

float B_madgwick = 0.05;  //(default 0.04)
float q0 = 1.0f;          //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

const int wheelRadius = 0.3;

void setup()
{
  Serial.begin(115200);
  encoder.begin();
  // initiate IMU
  IMUinit();
  //calculate_IMU_error();
}

void loop()
{
  tick(); // start timer
  Rpm_filt = computeActualRpm();
  // calculate IMU heading
  getIMUdata();
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);
  yaw_IMU = map(yaw_IMU, 0, 80, 0, 90);
  // Serial.print(yaw_IMU);
  // Serial.print("     ");
  Serial.println(String(int(Rpm_filt))+"/"+String(int(yaw_IMU)));

  tock(); // end timer
}

float computeActualRpm() {
  currentPosition = encoder.getCount();
  deg = map(currentPosition, 0, 1450, 0, 360);
  curPosRad = deg * (PI / 180);

  curTime = micros();
  deltaTimeEnc = (float)(curTime - prevTime) / 1.0e6;
  //Serial.println(deltaTimeEnc);
  //Serial.print(curPosRad);
  //Serial.print("    ");
  //Serial.println(prevPosRad);
  actualVelocity = (curPosRad - prevPosRad) / deltaTimeEnc;  // rad/sec
  prevActualRpm = actualRpm;
  actualRpm = actualVelocity * (60 / (2 * PI));  //rpm
  prevPosRad = curPosRad;
  prevTime = curTime;
  delay(3);
  Rpm_filt = 0.854 * Rpm_filt + 0.0728 * actualRpm + 0.0728 * prevActualRpm;
  return Rpm_filt;
}