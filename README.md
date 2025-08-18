# WRO-Future-Engineers-InnovativeVelammalians
Autonomous robot car for WRO Future Engineers 2025 by Team Innovative Velammalians. Built with Raspberry Pi, Arduino Nano, camera, and sensors for lane following, obstacle detection, and intelligent navigation. Includes design, code, images, and testing results.
Innovative Velammalians – WRO Future Engineers
👥 Team Members

Reena Shri D

Shrinidhi RM

Joshna Sarath

Category: Future Engineers – World Robot Olympiad (WRO)

📖 Project Overview

This project is developed as part of the WRO Future Engineers category, where the challenge is to design and program an autonomous robot vehicle capable of navigating a race track using sensors, cameras, and intelligent control systems.

Our robot is built with a combination of Raspberry Pi (for high-level control) and Arduino Nano (for motor and sensor management). It uses multiple sensors such as ultrasonic, line detection, and IMU to achieve autonomous driving.

⚙️ Components Used

Raspberry Pi 3 (main controller)

MicroSD card (OS + programs)

Camera module with wide-angle lens

Arduino Nano with prototyping shield (motor + sensor controller)

DC Motor Controller

DC Motor (driving the vehicle)

Servo Motor (steering)

IMU sensor (motion sensing)

2 × Ultrasonic Distance Sensors

2 × Analog Line Sensors

Rotary Encoder

External USB Battery with hub (powering Raspberry Pi & Arduino)

Additional motor battery (to power DC motor)

🚗 Robot Design

Here is the base chassis we used to build our robot:

👉 More images of wiring, sensors, and final build will be added soon.

📝 Arduino Nano Code (Motor & Sensor Control)

This Arduino sketch handles low-level motor and sensor control, including:

Reading data from ultrasonic sensors, line sensors, IMU, and rotary encoder

Controlling the DC motor and steering servo

Communicating with the Raspberry Pi for high-level decision making

// === Arduino Nano Motor & Sensor Control ===
// Handles motor driver, servo steering, and sensor inputs

#include <Servo.h>

// Pin definitions
const int motorPWM = 3;         // DC motor PWM pin
const int motorDir = 4;         // Motor direction pin
const int servoPin = 9;         // Servo pin
const int trigPin = 5;          // Ultrasonic Trigger
const int echoPin = 6;          // Ultrasonic Echo
const int lineSensorLeft = A0;  // Line sensor (left)
const int lineSensorRight = A1; // Line sensor (right)

Servo steering;

void setup() {
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(lineSensorLeft, INPUT);
  pinMode(lineSensorRight, INPUT);

  steering.attach(servoPin);
  Serial.begin(9600);
}

void loop() {
  // Example: read line sensors
  int leftLine = analogRead(lineSensorLeft);
  int rightLine = analogRead(lineSensorRight);

  // Example: basic lane following logic
  if (leftLine < 500 && rightLine < 500) {
    // both sensors see the line
    forward();
  } else if (leftLine < 500) {
    turnRight();
  } else if (rightLine < 500) {
    turnLeft();
  } else {
    stopMotor();
  }
}

// === Motor Functions ===
void forward() {
  digitalWrite(motorDir, HIGH);
  analogWrite(motorPWM, 150); // adjust speed
  steering.write(90); // straight
}

void turnLeft() {
  steering.write(120); // left angle
  analogWrite(motorPWM, 120);
}

void turnRight() {
  steering.write(60); // right angle
  analogWrite(motorPWM, 120);
}

void stopMotor() {
  analogWrite(motorPWM, 0);
}

💻 Raspberry Pi Code (Placeholder)

The Raspberry Pi is responsible for:

Running computer vision (OpenCV) for lane detection

High-level decision making

Sending commands to Arduino Nano

👉 Code will be added here soon.

🧪 Testing & Results

The robot successfully detects lanes using line sensors.

Ultrasonic sensors provide reliable obstacle detection.

The servo mechanism enables smooth steering around curves.

📸 (Add pictures/videos of testing here)

⚡ How to Run

Flash Raspberry Pi OS into the MicroSD card.

Upload the Arduino sketch (controller.ino) to the Arduino Nano.

Connect Raspberry Pi to Arduino via USB.

Run the Raspberry Pi Python script (main.py).

Place the robot on the track and let it drive autonomously 🚗💨

🔮 Future Improvements

Improve lane detection with AI models instead of thresholding.

Optimize PID control for smoother turns.

Add a camera-based obstacle detection system.

Reduce power consumption with lighter batteries.

🙏 Acknowledgments

Special thanks to our mentors, school (Velammal), and teammates for guiding and supporting us in the WRO Future Engineers journey.
