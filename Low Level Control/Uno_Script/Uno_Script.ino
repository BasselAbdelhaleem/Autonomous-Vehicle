//#include <SoftwareSerial.h>
#include <Servo.h>
//#include <Encoder.h>
//#define ENCODER_USE_INTERRUPTS

//Encoder myEnc(2, 3);

//const int rxPin = 7;
//const int txPin = 8;
//SoftwareSerial Serial1 (rxPin, txPin);

Servo servo;

long currentPosition;
float deg;
float curPosRad;
float prevPosRad;
float actualVelocity;
float actualRpm;
float prevActualRpm;
float Rpm_filt;
long time;
float deltaTime;
long prevTime;


bool led = false;

// Define motor control pins
const int motorPin1 = 11;  // Connect this to L EN on the motor driver
const int motorPin2 = 12;  // Connect this to R EN on the motor driver

// Define motor speed control pins
const int motorSpeedPin1 = 5;  // Connect this to the enable pin 1 of Motor
const int motorSpeedPin2 = 6;  // Connect this to the enable pin 2 of Motor


const int servoPin = 9;  // Connect steering servo to pin 9


// desired velocity & angle  message received from Raspberry Pi

String desiredData;
int PwmDes;
int angleDes;

void setup()
{
  Serial.begin(115200);

  // Software Serial pins & baudrate
  //pinMode(rxPin, INPUT);
  //pinMode(txPin, OUTPUT);
  //Serial1.begin(115200);

  // Initialize motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorSpeedPin1, OUTPUT);
  pinMode(motorSpeedPin2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  servo.attach(servoPin);
}

void loop()
{
  // Read the incoming data
  //String data = Serial.readStringUntil('\r\n');

  //String velocityDes = data.substring(0, data.indexOf('/'));
  //String angleDes = data.substring(data.indexOf('/') + 1);
  
  //setTargetVelocity(velocityDes.toInt());
  if(Serial.available()>0){
    desiredData = Serial.readStringUntil('\r\n');
    // Split the received data into velocity & angle
    PwmDes = desiredData.substring(0, desiredData.indexOf('/')).toInt();
    //PwmDes=100;
  }

  steer(PwmDes);

  //Rpm_filt = computeActualRpm();

  //Serial1.println(Rpm_filt);

//  Serial.print(-100);
//  Serial.print("\t");
//  Serial.print("\t");
//  Serial.print(Rpm_filt);
//  Serial.print("\t");
//  Serial.print(100);
//  Serial.println(angleDes);
    
}
/*
float computeActualRpm() {
  currentPosition = myEnc.read();
  deg = map(currentPosition, 0, 1450, 0, 360);
  curPosRad = deg * (PI / 180);

  time = micros();
  deltaTime = ((float)(time - prevTime)) / 1.0e6;
  //Serial.println(deltaTime);
  //Serial.print(curPosRad);
  //Serial.print("    ");
  //Serial.println(prevPosRad);
  actualVelocity = (curPosRad - prevPosRad) / deltaTime;  // rad/sec
  prevActualRpm = actualRpm;
  actualRpm = actualVelocity * (60 / (2 * PI));  //rpm
  prevPosRad = curPosRad;
  prevTime = time;
  delay(3);
  Rpm_filt = 0.854 * Rpm_filt + 0.0728 * actualRpm + 0.0728 * prevActualRpm;
  return Rpm_filt;
}
*/
void moveForward(int motorSpeed) {
  //Serial.print("Moving forward: ");
  //Serial.println(motorSpeed);

  // Set motor directions for forward motion
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);

  // Set PWM value to control motor speed (255 out of 255 maximum)
  analogWrite(motorSpeedPin1, motorSpeed);
  analogWrite(motorSpeedPin2, motorSpeed);
}

void moveBackward(int motorSpeed) {
  //Serial.print("Moving backward: ");
  //Serial.println(motorSpeed);


  // Set motor directions for backward motion
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  // Set PWM value to control motor speed (150 out of 255 maximum)
  analogWrite(motorSpeedPin1, motorSpeed);
  analogWrite(motorSpeedPin2, motorSpeed);
}

void steer(int angle) {
  angle = map(angle, -45, 45, 45, 135);  // map input angle to servo angle range
  //Serial.print("Steering: ");
  Serial.println(angle);
  servo.write(angle);
}