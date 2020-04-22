 /*
 * Library: https://github.com/bolderflight/MPU9250
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * Updated by Ahmad Shamshiri on July 09, 2018 for Robojax.com
 * in Ajax, Ontario, Canada
 * watch instrucion video for this code: 
For this sketch you need to connect:
VCC to 5V and GND to GND of Arduino
SDA to A4 and SCL to A5

S20A is 3.3V voltage regulator MIC5205-3.3BM5
*/

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU_Finger(Wire,0x68);
MPU9250 IMU_Hand(Wire,0x69);
int status_hand;
int status_finger;

//String dataString = "";

int buttonPin = D3;
int ledPin = D4;
int prevButtonState = 0;
int buttonState = 0;
int ledState = 0;
int lastChangeTime = millis();

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with Hand IMU 
  status_hand = IMU_Hand.begin();
  if (status_hand < 0) {
    Serial.println("Hand IMU initialization unsuccessful");
    Serial.println("Check Hand IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status_hand);
    while(1) {}
  }

  // start communication with Finger IMU 
  status_finger = IMU_Finger.begin();
  if (status_finger < 0) {
    Serial.println("Finger IMU initialization unsuccessful");
    Serial.println("Check Finger IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status_finger);
    while(1) {}
  }

  // setting the accelerometer full scale range to +/-2G 
  IMU_Hand.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU_Finger.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU_Hand.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU_Finger.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU_Hand.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU_Finger.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU_Hand.setSrd(19);
  IMU_Finger.setSrd(19);

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  //Particle.variable("DataString", dataString);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // read the sensors
  IMU_Hand.readSensor();
  double hand_read_time = millis();

  IMU_Finger.readSensor();
  double finger_read_time = millis();

  double ax_hand = IMU_Hand.getAccelX_mss();
  double ay_hand = IMU_Hand.getAccelY_mss();
  double az_hand = IMU_Hand.getAccelZ_mss();

  double gx_hand = IMU_Hand.getGyroX_rads();
  double gy_hand = IMU_Hand.getGyroY_rads();
  double gz_hand = IMU_Hand.getGyroZ_rads();

  double gy_finger = IMU_Finger.getGyroY_rads();

  //Serial.println(buttonState);

  if (buttonState == 1 && (millis() - lastChangeTime > 1000)) {
    lastChangeTime = millis();

    if (ledState == 0) {
      // Begin recording data
      
      digitalWrite(ledPin, HIGH);
      ledState = 1;
    } else {
      // Stop recording data

      digitalWrite(ledPin, LOW);
      ledState = 0;
    }
  }

  if (ledState == 1) {
    // Record data
    Particle.publish("ax", String(ax_hand));
    Particle.publish("ay", String(ay_hand));
    Particle.publish("az", String(az_hand));

    int throwState = 0;

    // If we sense a throw, set throwState = 1
    if (gy_finger <= -4) {
      throwState = 1;
    }

    Serial.print(throwState);
    Serial.print(",");
    Serial.print(hand_read_time);
    Serial.print(",");
    Serial.print(ax_hand,6);
    Serial.print(",");
    Serial.print(ay_hand,6);
    Serial.print(",");
    Serial.print(az_hand,6);
    Serial.print(",");
    Serial.print(gx_hand,6);
    Serial.print(",");
    Serial.print(gy_hand,6);
    Serial.print(",");
    Serial.println(gz_hand,6);
  }

  prevButtonState = buttonState;

  delay(20);
} 
