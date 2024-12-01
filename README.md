# line-following-robot

To create a GitHub-friendly README file for your Arduino project, you can provide a clear and structured overview with code snippets and explanations. Here's a formatted README template for your project:

---

# Arduino Line Following and Obstacle Avoidance Robot

This project demonstrates an Arduino-based robot that combines line-following capabilities with basic obstacle avoidance using IR and ultrasonic sensors.

## Features
- **Line Following**: The robot uses QTR sensors to detect the line and follow it using PID control.
- **Obstacle Avoidance**: IR sensors detect left and right obstacles, and the robot adjusts its direction accordingly.

## Components Used
1. **Arduino Board**
2. **QTR Sensors** (Analog, 4 sensors)
3. **Ultrasonic Sensors** (HC-SR04)
4. **IR Sensors** (Left and Right)
5. **Motor Driver (L298N)** or equivalent
6. **DC Motors**
7. **Power Supply**

## Pin Configuration
### Ultrasonic Sensors
| Sensor | Trigger Pin | Echo Pin |
|--------|-------------|----------|
| Front  | 8           | 9        |
| Left   | 10          | 11       |
| Right  | 12          | 13       |

### Motors
| Motor | Direction Pins | PWM Pin |
|-------|----------------|---------|
| A     | 2, 4           | 3       |
| B     | 7, 6           | 5       |

### QTR Sensors
| Pin | Function |
|-----|----------|
| A2  | Sensor 1 |
| A3  | Sensor 2 |
| A4  | Sensor 3 |
| A5  | Sensor 4 |

### IR Sensors
| Sensor | Pin  |
|--------|------|
| Left   | A1   |
| Right  | A0   |

## Code Overview

### PID Control for Line Following
The robot uses a Proportional-Derivative-Integral (PID) control mechanism to follow the line detected by QTR sensors. The PID values (`Kp`, `Kd`, `Ki`) are tuned for optimal performance.

```cpp
P = error;
D = error - previousError;
I = I + error;

PIDvalue = (Kp * P) + (Kd * D);
previousError = error;

lsp = lfspeed - PIDvalue;
rsp = rfspeed + PIDvalue;
```

### Motor Control
The robot adjusts motor speeds based on the PID value and the obstacle detection inputs.

```cpp
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
```

### Obstacle Avoidance
The robot changes direction when an obstacle is detected using IR sensors.

```cpp
if (analogRead(irLeft) > 500 && analogRead(irRight) < 500) {
  motorControl(-150, 150); // Turn left
  delay(300);
} else if (analogRead(irLeft) < 500 && analogRead(irRight) > 500) {
  motorControl(150, -150); // Turn right
  delay(300);
}
```

## Setup and Calibration
1. Connect all components as per the pin configuration table.
2. In the `setup` function, QTR sensors are calibrated to determine the min and max values for each sensor:
   ```cpp
   for (int i = 0; i < 300; i++) {
     qtr.calibrate();
     delay(20);
   }
   ```

3. Use the `linefollow` function for line-following behavior and `motorControl` for controlling the motors.

## Dependencies
- [QTRSensors Library](https://github.com/pololu/qtr-sensors-arduino)

## How to Run
1. Upload the code to your Arduino board.
2. Place the robot on a track with a line and test the behavior.
3. Adjust `Kp`, `Kd`, and `Ki` if needed to improve PID performance.

## Demo Video
*(Include a link or embed a video showing the robot in action.)*

## License
This project is open-source under the MIT License.

---

![WhatsApp Image 2024-12-01 at 12 27 24_6089a548](https://github.com/user-attachments/assets/b6c24d18-caa4-4997-81bb-c4307129fcc2)
![WhatsApp Image 2024-12-01 at 12 27 24_d5b087ff](https://github.com/user-attachments/assets/acb2ca21-4c66-4990-8652-ce995d8d6c17)




You can copy this content into a `README.md` file and include any additional details or images for better presentation.



