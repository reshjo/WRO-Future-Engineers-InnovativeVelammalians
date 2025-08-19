# WRO-Future-Engineers-InnovativeVelammalians
Autonomous robot car for WRO Future Engineers 2025 by Team Innovative Velammalians. Built with Raspberry Pi, Arduino Nano, camera, and sensors for lane following, obstacle detection, and intelligent navigation. Includes design, code, images, and testing results.

#  Innovative Velammalians – WRO Future Engineers

##  Team Members

* **Reena Shri D**
* **Shrinidhi RM**
* **Joshna Sarath**

**Category:** Future Engineers – World Robot Olympiad (WRO)

---

##  Project Overview

This project is developed as part of the **WRO Future Engineers category**, where the challenge is to design and program an **autonomous robot vehicle** capable of navigating a race track using sensors, cameras, and intelligent control systems.

Our robot is built with a combination of **Raspberry Pi (for high-level control)** and **Arduino Nano (for motor and sensor management)**. It uses multiple sensors such as ultrasonic, line detection, and IMU to achieve autonomous driving.

---

## ⚙️ Components Used

* **Raspberry Pi 3** (main controller)
* **MicroSD card** (OS + programs)
* **Camera module with wide-angle lens**
* **Arduino Nano with prototyping shield** (motor + sensor controller)
* **DC Motor Controller**
* **DC Motor** (driving the vehicle)
* **Servo Motor** (steering)
* **IMU sensor** (motion sensing)
* **2 × Ultrasonic Distance Sensors**
* **2 × Analog Line Sensors**
* **Rotary Encoder**
* **External USB Battery with hub** (powering Raspberry Pi & Arduino)
* **Additional motor battery** (to power DC motor)

---

##  Robot Design

Here is the base robot design image:

![Robot Chassis](assets/images/chassis.png)

 More images of wiring, sensors, and final build will be added soon.

---

## Arduino Nano Code (Motor & Sensor Control)

Below is the **complete Arduino code** we developed for motor control, steering, and sensor integration:

```cpp
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

// Define pins for ultrasonic sensors
#define TRIGGER_PIN1 2
#define ECHO_PIN1 3
#define TRIGGER_PIN2 4
#define ECHO_PIN2 5
#define MAX_DISTANCE 200

// Motor pins
#define ENA 9
#define IN1 7
#define IN2 8

// Servo pin
#define SERVO_PIN 6

// Line sensors
#define LINE_SENSOR_LEFT A0
#define LINE_SENSOR_RIGHT A1

// Rotary encoder
#define ENCODER_A 10
#define ENCODER_B 11

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
Servo steering;
Adafruit_MPU6050 mpu;

volatile int encoderCount = 0;

void encoderISR() {
  encoderCount++;
}

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
  // Ultrasonic distance
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();

  // Line sensors
  int leftLine = analogRead(LINE_SENSOR_LEFT);
  int rightLine = analogRead(LINE_SENSOR_RIGHT);

  // IMU data
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

  // Basic navigation logic
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
```

---

## 💻 Raspberry Pi Code (Placeholder)

The Raspberry Pi handles:

* Running **computer vision (OpenCV)** for lane detection
* High-level decision making
* Sending commands to Arduino Nano

👉 Code will be added here soon.

---

## Testing & Results

* The robot successfully **detects lanes** using line sensors.
* Ultrasonic sensors provide reliable **obstacle detection**.
* The servo mechanism enables **smooth steering** around curves.

📸 (Add pictures/videos of testing here)

---

## How to Run

1. Flash Raspberry Pi OS into the **MicroSD card**.
2. Upload the Arduino sketch (`controller.ino`) to the **Arduino Nano**.
3. Connect Raspberry Pi to Arduino via USB.
4. Run the Raspberry Pi Python script (`main.py`).
5. Place the robot on the track and let it drive autonomously 🚗💨

---

## 🔮 Future Improvements

* Improve **lane detection with AI models** instead of thresholding.
* Optimize **PID control** for smoother turns.
* Add a **camera-based obstacle detection** system.
* Reduce power consumption with lighter batteries.

---

## Acknowledgments

Special thanks to our mentors, school (Velammal), and teammates for guiding and supporting us in the WRO Future Engineers journey
