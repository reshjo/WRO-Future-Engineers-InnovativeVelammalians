WRO Future Engineers – Innovative Velammalians
Project Overview

This repository documents the autonomous vehicle prototype developed by Team Innovative Velammalians for the WRO 2025 Future Engineers Category.
Our vehicle is designed to navigate, detect obstacles, and adapt its driving path using a combination of Arduino Nano (for low-level motor & sensor control) and Raspberry Pi 3 (for high-level decision making).

Team Members

Reena Shri D

Shrinidhi RM

Joshna Sarath

Hardware Components

Main Controller: Raspberry Pi 3 with MicroSD card (OS + Programs)

Motor & Sensor Controller: Arduino Nano with prototyping shield

Camera Module: Wide-angle lens

DC Motor & Motor Driver (L298N)

Servo Motor for steering

IMU Sensor (MPU6050)

2 Ultrasonic Sensors for distance detection

2 Analog Line Sensors for lane detection

Rotary Encoder for wheel movement tracking

External USB Battery Pack (powering Raspberry Pi & Arduino)

Additional Battery for motor supply

System Architecture

Arduino Nano handles:

Ultrasonic sensors

IMU readings

Line sensors

Encoder counting

Direct motor actuation & steering servo

Raspberry Pi handles:

Higher-level navigation logic

Obstacle avoidance strategy

Control signals to Arduino

LED status indicators and button input

Features

Obstacle detection & avoidance using ultrasonic sensors

Smooth servo steering with calibrated center trim

Line following via analog sensors

Encoder-based speed/distance tracking

IMU-based motion stability check

Autonomous navigation with safety stop mechanism

Multi-power supply system (dedicated motor & logic power)

Arduino Nano Code (Low-Level Control)
// Arduino Nano Code
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#define TRIGGER_PIN1 2
#define ECHO_PIN1 3
#define TRIGGER_PIN2 4
#define ECHO_PIN2 5
#define MAX_DISTANCE 200

#define ENA 9
#define IN1 7
#define IN2 8

#define SERVO_PIN 6

#define LINE_SENSOR_LEFT A0
#define LINE_SENSOR_RIGHT A1

#define ENCODER_A 10
#define ENCODER_B 11

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
Servo steering;
Adafruit_MPU6050 mpu;

volatile int encoderCount = 0;

void encoderISR() { encoderCount++; }

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  steering.attach(SERVO_PIN);

  pinMode(LINE_SENSOR_LEFT, INPUT);
  pinMode(LINE_SENSOR_RIGHT, INPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
}

void loop() {
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();

  int leftLine = analogRead(LINE_SENSOR_LEFT);
  int rightLine = analogRead(LINE_SENSOR_RIGHT);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Distances: ");
  Serial.print(distance1);
  Serial.print(" cm, ");
  Serial.print(distance2);
  Serial.println(" cm");

  Serial.print("Line Sensors: L=");
  Serial.print(leftLine);
  Serial.print(" R=");
  Serial.println(rightLine);

  Serial.print("IMU Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" Z: ");
  Serial.println(a.acceleration.z);

  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);

  if (distance1 > 20 && distance2 > 20) {
    moveForward(150);
  } else {
    stopMotors();
  }

  delay(100);
}

void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  steering.write(90);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

Raspberry Pi Code (High-Level Navigation)
// Raspberry Pi C++ Code using pigpio
#pragma GCC optimize("Ofast")
#pragma GCC optimize("unroll-loops")

#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>

// Pin definitions, parameters, helper functions here (LED, Motors, Servo, Ultrasonic)...

int main() {
  setup_gpio();

  // Idle: blue LED, wait for button press
  LED_rgb(0, 0, 255);
  int btn_cnt = 0;
  while (btn_cnt < 10) { if (is_button_down()) btn_cnt++; else btn_cnt = 0; sleep_ms(20); }

  // Start autonomous navigation
  LED_rgb(0, 255, 0);
  servo_set_angle(0);
  sleep_ms(150);

  while (true) {
    float d = median_distance_cm(3, 20);
    if (d < AVOID_DIST_CM) {
      // Obstacle avoidance logic: scan left/right, reverse if necessary
      // Uses servo sweep + motor control
    }
    else {
      // Clear path: move forward
      set_motors(CRUISE_SPEED, CRUISE_SPEED);
    }
    sleep_ms(15);
  }

  // Shutdown
  motors_stop();
  servo_set_angle(0);
  LED_rgb(0, 0, 255);
  gpioTerminate();
  return 0;
}

Images

 Conclusion 

Limitations 

Obstacle avoidance is limited to stopping only. 

Line sensors sensitive to lighting changes. 

Additional battery adds weight. 

 
