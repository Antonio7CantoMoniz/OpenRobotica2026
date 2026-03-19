# OpenRobotica2026 - High-Speed Line Follower Robot 🏎️💨

This repository contains the C++ firmware for an ESP32-based high-speed line-follower robot. The project was developed with a focus on raw speed, precise control, and robust track-fault tolerance, making it suitable for competitive robotics.

## 🌟 Key Engineering Features

* **Direct-Math PID Control:** Unlike standard weighted-average algorithms, this code uses a Direct-Math approach for the Proportional-Derivative (PD) controller (`Kp = 6.0`, `Kd = 7.0`). This provides maximum turning torque exactly when the sensors detect a track deviation, eliminating "lazy" sine-wave oscillations.
* **Channel Separation Logic:** To prevent false finish-line detections at high speeds (the "Levitation Effect"), the sensor array tasks are physically separated in the code:
    * **Steering (PID):** Exclusively uses the inner sensors (`S3`, `S4`, `S5`).
    * **Finish Line Detection:** Exclusively uses the extreme outer sensors (`S1` and `S5`), making it physically impossible to trigger a false stop mid-track.
* **Pre-Race Auto-Calibration:** Features a pre-race sequence triggered by an LDR (Light Dependent Resistor). Before launch, the robot reads the track's specific lighting conditions and calculates the exact threshold between black and white in real-time.
* **Fail-Safe "Survival" Mode:** If the robot completely loses the line at high speed (all sensors read white), the system accesses its memory (`ultimo_erro`) to determine the last known track position. It then applies a maximum override torque (Error = 5.0) to violently snap the robot back onto the track. If recovery fails after 1 second, the motors shut down to prevent hardware damage.
* **Start-Line Blindfold Timer:** Incorporates a `millis()` timer logic that actively ignores transversal start lines during the first 4 seconds of acceleration, preventing premature motor cut-offs.

## 🛠️ Hardware Requirements

* **Microcontroller:** ESP32 (Bluetooth Serial enabled for debugging)
* **Sensors:** 5-Array IR Sensor module + 1 LDR (for launch sequence)
* **Motor Driver:** Standard H-Bridge (e.g., L298N or TB6612FNG)
* **Motors:** 2x DC Motors

## 🚀 How to Run (Race Protocol)

1. Place the robot behind the start line.
2. Cover the LDR sensor (The calibration LED will turn ON).
3. Perform the "Calibration Dance": slide the front sensor array across the black line and the white floor.
4. Center the robot perfectly on the line.
5. Uncover the LDR to launch!
