#include <QTRSensors.h>

// Ultrasonic Sensor Pins
#define TRIG1 8
#define ECHO1 9
#define TRIG2 10
#define ECHO2 11
#define TRIG3 12
#define ECHO3 13

// Motor pins
const int in1PinA = 2;  // Motor A
const int in2PinA = 4;
const int enablePinA = 3;  // PWM for Motor A

const int in1PinB = 7;  // Motor B
const int in2PinB = 6;
const int enablePinB = 5;  // PWM for Motor B

//speed 80- 0.03
//speed 100 - 0.05
//spped 120 - 0.06

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 100;
int rfspeed = 100;

float Kp = 0.05;
float Kd = 0.0001;
float Ki = 0;


// float Kp = 0.07;
// float Kd = 0.000;
// float Ki = 0;

// QTR Sensors configuration
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

// Distance thresholds
const int WALL_DISTANCE_FRONT = 20;  // Threshold distance for front wall (cm)
const int WALL_DISTANCE_SIDE = 15;   // Threshold distance for side walls (cm)

//IR Left and Right
const int irLeft = A1;
const int irRight = A0;

void setup() {

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A2, A3, A4, A5 }, SensorCount);

  // Set motor direction pins as output
  pinMode(in1PinA, OUTPUT);
  pinMode(in2PinA, OUTPUT);
  pinMode(in1PinB, OUTPUT);
  pinMode(in2PinB, OUTPUT);

  // Initialize ultrasonic sensors
  // pinMode(TRIG1, OUTPUT);
  // pinMode(ECHO1, INPUT);
  // pinMode(TRIG2, OUTPUT);
  // pinMode(ECHO2, INPUT);
  // pinMode(TRIG3, OUTPUT);
  // pinMode(ECHO3, INPUT);

  //Ir
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  Serial.begin(115200);

  // Calibrate line sensor
  Serial.println("Calibration start.......");
  for (int i = 0; i < 300; i++) {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibration complete.");
}

void loop() {
  // Read distances
  // double frontDistance = getUltrasonicDistance(TRIG1, ECHO1);
  // double rightDistance = getUltrasonicDistance(TRIG2, ECHO2);
  // double leftDistance = getUltrasonicDistance(TRIG3, ECHO3);

  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print(qtr.calibrationOn.minimum[i]);
  //   Serial.print('\t');
  // }

   if (analogRead(irLeft) > 500 && analogRead(irRight) < 500) {
    // Turn right condition
    Serial.println("Turn left...");
    // Adjust delay based on track width
    motorControl(-150, 150);
    delay(300);

  } else if (analogRead(irLeft) < 500 && analogRead(irRight) > 500) {
    // Turn left condition
    Serial.println("Turn right...");
    // Adjust delay based on track width
    motorControl(150, -150);
    delay(300);
  } else {
    Serial.println("Turn line...");
    linefollow();  // Call line following logic
  }

  // linefollow();
}

void motorControl(int leftSpeed, int rightSpeed) {
  // Motor A control
  if (leftSpeed > 0) {
    digitalWrite(in1PinA, HIGH);
    digitalWrite(in2PinA, LOW);
  } else {
    digitalWrite(in1PinA, LOW);
    digitalWrite(in2PinA, HIGH);
  }
  analogWrite(enablePinA, abs(leftSpeed));

  // Motor B control
  if (rightSpeed > 0) {
    digitalWrite(in1PinB, HIGH);
    digitalWrite(in2PinB, LOW);
  } else {
    digitalWrite(in1PinB, LOW);
    digitalWrite(in2PinB, HIGH);
  }
  analogWrite(enablePinB, abs(rightSpeed));
}

void linefollow() {
  // Read calibrated line sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println("");

  // Serial.print("Position: ");
  // Serial.println(position);

  error = position - 1500;  // Assume center position is 2500 for 6 sensors

  Serial.print("error: ");
  Serial.println(error);

  P = error;
  D = error - previousError;
  I = I + error;

  PIDvalue = (Kp * P) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = rfspeed + PIDvalue;

  // lsp = constrain(lsp, 10, 255);
  // rsp = constrain(rsp, 10, 255);

  if (lsp > 255) {
    lsp = 255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (lsp < 0) {
    lsp = 0;
  }

  // Serial.print("lsp: ");
  // Serial.println(lsp);
  // Serial.print("rsp: ");
  // Serial.println(rsp);

  motorControl(lsp, rsp);
  // delay(500);
}

// double getUltrasonicDistance(int trigPin, int echoPin) {
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
//   long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
//   double distance = duration * 0.034 / 2;         // Convert to cm
//   return distance;
// }
