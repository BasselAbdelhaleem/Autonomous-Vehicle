
void IMUinit() {
  //DESCRIPTION: Initialize IMU
  if (!IMU.begin()) {
    while (true)
      ;
  }
  delay(15);
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU

  //Accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccY = AccY - AccErrorZ;
    //Correct the outputs with the calculated error values
    /*
    //This was used to suppress noise.  However, it just slows things down.  There is already internal filters to the IMU.
    AccX = (1.0 - Accel_filter) * AccX_prev + Accel_filter * AccX;
    AccY = (1.0 - Accel_filter) * AccY_prev + Accel_filter * AccY;
    AccZ = (1.0 - Accel_filter) * AccZ_prev + Accel_filter * AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;
    */
  } else return;
  if (IMU.gyroscopeAvailable()) {
    //Gyro
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
    /*
    //This was used to suppress noise.  However, it just slows things down.  There is already internal filters to the IMU.
    GyroX = (1.0 - Gyro_filter) * GyroX_prev + Gyro_filter * GyroX;
    GyroY = (1.0 - Gyro_filter) * GyroY_prev + Gyro_filter * GyroY;
    GyroZ = (1.0 - Gyro_filter) * GyroZ_prev + Gyro_filter * GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
    */
  }
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltaTime;
  q1 += qDot2 * deltaTime;
  q2 += qDot3 * deltaTime;
  q3 += qDot4 * deltaTime;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951;  //degrees
  pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                 //degrees
  yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951;  //degrees
}

void tick() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  deltaTime = (current_time - prev_time) / 1000000.0;  //division takes it from micros to seconds.  1000000 ms=1 second
}

void tock() {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. I have matched 
   * it just below the Gyro update frequency.  invFreq is set at the top of the code.
   */
  //Sit in loop until appropriate time has passed while checking for any WiFi client activity.
  while (invFreq > (micros() - current_time)) {
    //if (ALLOW_WIFI&&throttle_is_cut) loopWiFi();
  };
}

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0 / sqrtf(x);  //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata().
   * this will correct for sensor drift and crookedness in your copter.
   */
  float AcX, AcY, AcZ, GyX, GyY, GyZ;

  Serial.println("Calculating IMU Error with 10000 iterations. Please stand by...");

  //Read IMU values 1000 times.  Should take around 2 minutes at 104Hz IMU timing.
  int c = 0;AccErrorX=0;AccErrorY=0;AccErrorZ=0;GyroErrorX=0;GyroErrorY=0;GyroErrorZ=0;
  while (c < 10000) {
    while (!IMU.accelerationAvailable()&!IMU.gyroscopeAvailable()) delay(1);
    IMU.readAcceleration(AcX, AcY, AcZ);
    IMU.readGyroscope(GyX, GyY, GyZ);
    AccX = AcX;
    AccY = AcY;
    AccZ = AcZ;
    GyroX = GyX;
    GyroY = GyY;
    GyroZ = GyZ;

    //Sum all readings
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
    Serial.println(String(10000-c));
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX = AccErrorX / c;
  AccErrorY = AccErrorY / c;
  AccErrorZ = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");

  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
  while (true) delay(1000);
}