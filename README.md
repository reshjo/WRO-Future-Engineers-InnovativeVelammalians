

 # WRO Future Engineers – Innovative Velammalians

This repository documents our team's entry for the **World Robot Olympiad – Future Engineers** category. Our autonomous self-driving prototype demonstrates hardware-software integration, timed steering, and novel chassis design.

---

## Team

**Team Name:** *Innovative Velammalians*

**Members:**

* Reena Shri D
* Shrinidhi RM
* Joshna Sarath

**Coach:** Mrs. Reena Manopriya

---

## Hardware Components

* Arduino Uno – Main microcontroller
* L298N Motor Driver – Controls DC motors
* DC Motors – Drive propulsion
* Servo Motor – Steering mechanism
* Li-Po Battery – Power supply
* Custom CAD-designed chassis

---

## Chassis Design

### 3D Perspective View

<img src="IMG_0263.jpeg" alt="3D Chassis View" width="600"/> 

### Top View

<img src="IMG_0261.jpeg" alt="Chassis Top View" width="600"/> 

### Bottom View

<img src="IMG_0260.jpeg" alt="Chassis Bottom View" width="600"/> 

---

## Demonstration Video

[Watch on YouTube](https://youtu.be/1QLYZ_jNva8?si=0_mQt1Ktheclp281)

---

## Source Code

The Arduino sketch controlling the DC motor and servo steering is provided below. It executes timed right turns 12 times, followed by a final forward motion before stopping.

**File:** `code/main.ino`

```cpp
#include <AFMotor.h>
#include <Servo.h>

// DC Motor on M3
AF_DCMotor motor(3); 
Servo steering;

// Timing and logic
unsigned long previousMillis = 0;
const unsigned long interval = 5000;  // 5 seconds between turns
bool turning = false;
int turnCount = 0;
const int maxTurns = 12;
bool finalForwardDone = false;
bool started = false;
bool waitingForTrigger = true;

// Button pin
const int triggerPin = A5;

void setup() {
  // Start Serial Monitor
  Serial.begin(9600);
  Serial.println("Robot Initialized. Waiting for button press on A5...");

  pinMode(triggerPin, INPUT_PULLUP);  // Use internal pull-up resistor

  motor.setSpeed(255);       
  steering.attach(9);
  steering.write(90);  // Center
}

void loop() {
  // Wait for button press (LOW when pressed due to INPUT_PULLUP)
  if (waitingForTrigger) {
    if (digitalRead(triggerPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(triggerPin) == LOW) {
        Serial.println("Button press detected! Starting motion...");
        started = true;
        waitingForTrigger = false;
        motor.run(FORWARD);
        previousMillis = millis();
      }
    }
    return; // Wait until button is pressed
  }

  if (!started) return;

  unsigned long currentMillis = millis();

  if (!turning && currentMillis - previousMillis >= interval && turnCount < maxTurns) {
    previousMillis = currentMillis;
    turning = true;

    turnCount++;
    Serial.print("Turn #");
    Serial.print(turnCount);
    Serial.println(" → Turning right...");

    // Turn right
    steering.write(30);
    delay(1500);  // Hold for 1.5s

    // Back to center
    steering.write(90);
    Serial.println("Returned to center.");
    turning = false;
  }

  // After 12 turns, move straight for 2 sec and stop
  if (turnCount >= maxTurns && !finalForwardDone) {
    Serial.println("Completed 12 turns. Moving forward for 2 more seconds...");
    delay(2000);                // Final forward motion
    motor.run(RELEASE);         // Stop motor
    steering.detach();          // Power off servo
    Serial.println("Robot stopped completely.");
    finalForwardDone = true;
  }
}
```

---

## Execution Instructions

1. **Hardware Setup**

   * Assemble the chassis and mount the DC motors, servo motor, and driver module.
   * Connect components according to the wiring schematic.
   * Power the system using a Li-Po battery.

2. **Software Setup**

   * Install the Arduino IDE.
   * Add required libraries: `AFMotor.h` and `Servo.h`.
   * Upload the sketch from `code/main.ino` to the Arduino Uno.

3. **Operation**

   * Place the vehicle on a flat surface.
   * Press the trigger button connected to pin **A5** to start motion.
   * The robot will execute 12 timed right turns and then move forward for 2 seconds before stopping.

---

## Project Roadmap

* Completed: CAD chassis design
* Completed: Hardware assembly
* Completed: Initial motion testing
* In Progress: Vision & sensor integration
* Pending: Final optimization and competition readiness

---

## Acknowledgements

We gratefully acknowledge:

* The **World Robot Olympiad** organizers
* Our coach and institution for technical guidance
* Our team members for their dedication and collaboration



Attachments area
Preview YouTube video WRO Future Engineers | Open Challenge | 2025



