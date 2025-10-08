| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# WHEEL-MOVEMENT: FreeRTOS-Based Wheel Control System

This is capable of moving a wheel in a certain velocity and distance. This project implements the AS5600 magnetic encoder, the VL53L1X long range LiDAR and the TM151 9-axis IMU. Nevertheless, at the time, only the encoder is being used for something useful.



---

## 🛠️ Key Features

* **Real-Time Control:** Utilizes **FreeRTOS** for prioritized task management, ensuring stability and fast response in motor control and sensor data acquisition.
* **Velocity PID:** Implements a **PID control loop** for each wheel, based on readings from the **AS5600 encoders** to maintain the desired speed.
* **Interactive Calibration:** Allows real-time calibration of PID parameters (`Kp`, `Ki`, `Kd`) and the set-point via serial communication (see `pid_calibration.c`).
* **Sensor Fusion:** Algorithms to combine data from IMU, Encoder, and ToF (Lidar) for a more accurate and noise-resistant velocity and position estimation.
* **Encoder Testing:** Dedicated code (see `encoder_test.c`) for initialization and testing the reading of the angle (`AS5600_ADC_GetAngle`) and velocity estimation of the AS5600 encoder.

---

## 📁 Project Structure

The project structure follows a modular organization for managing drivers, core logic, and sensors.
```
WHEEL-MOVEMENT/
├── drivers/
│   ├── bldc_pwm/
│   │   ├── include/
│   │   └── bldc_pwm.c
│   ├── pid_lib/
│   │   ├── include/
│   │   ├── pid_ext.h
│   │   └── pid_ext.c
│   └── sensor_fusion/
│       ├── include/
│       └── sensor_fusion.c
├── main/
│   ├── CMakeLists.txt
│   ├── pid_calibration.c
│   ├── encoder_test.c
│   └── main.c
├── platform/
│   ├── include/
│   └── platform_esp32s3.c
├── sensors/
│   ├── AS5600_HAL_Library/
│   │   ├── include/
│   │   └── as5600_lib.c
│   ├── TM151-driver/
│   │   ├── include/
│   │   ├── EasyObjectDictionary.c
│   │   ├── EasyProfile.c
│   │   ├── EasyProtocol.c
│   │   ├── EasyQueue.c
│   │   └── EasyRetrieve.c
│   └── VL53L1X-driver/
│       ├── VL53L1X.c
├── CMakeLists.txt
├── README.md
└── sdkconfig
```

---

## 📝 Usage and Calibration

### PID Controller Calibration

The `pid_calibration.c` file implements a task (`uart_task`) that listens on the serial port to receive commands and update PID parameters in real-time.

**Serial Command:**
To update the parameters $K_p$, $K_i$, $K_d$ and the `set_point`, send the following command:

```
P <Kp> <Ki> <Kd> <SetPoint>
```


**Example:** `P 1.5 0.1 0.05 10.0`

This results in: `K_p = 1.5`, `K_i = 0.1`, `K_d = 0.05`, and a target velocity (`set_point`) of `10.0` (likely rad/s or m/s, depending on the implementation).

### AS5600 Encoder Test

The `encoder_test.c` file shows the initialization and use of the AS5600 encoder:
1.  **Configuration:** Sets the `CONF` register configuration (`AS5600_SetConf`). In `main.c`, it is configured for **Analog output (10%-90% RR)**, **Watchdog ON**, and specific filters.
2.  **Range Calibration:** Start position (`AS5600_SetStartPosition`) and stop position (`AS5600_SetStopPosition`) are configured.
3.  **Angle and Velocity Reading:** The `control_task` continuously reads the angle (`AS5600_ADC_GetAngle`) and uses `estimate_velocity_encoder` to calculate the velocity.

---

## 🧠 `sensor_fusion.c` – Velocity & Position Estimation

This module implements lightweight sensor fusion algorithms for real-time state estimation, ensuring more stable and accurate readings.

### Key Functions:

#### `estimate_velocity_imu(...)`
- Estimates velocity by trapezoidal integration of IMU acceleration.
- Includes exponential smoothing for noise rejection.

#### `estimate_velocity_encoder(...)`
- Calculates velocity from AS5600 encoder angle deltas and wheel radius.
- Applies a low-pass filter and ignores micro noise below a threshold.

#### `estimate_velocity_lidar(...)`
- Calculates velocity from distance differences measured by the ToF sensor.
- Filters out noisy variations and updates velocity using a moving average.

#### `estimate_position(...)`
- Returns the average position between lidar and encoder estimates.

#### `estimate_velocity(...)`
- **Robust Fusion:** Combines IMU, encoder, and lidar velocities. Chooses the pair with the least deviation to average, achieving a more robust estimation.

---

## ⚙️ PID Control Strategy

Each wheel is controlled through a **PID feedback loop** based on encoder readings. Velocity commands are generated, and sensor fusion is used to improve accuracy in noisy environments. FreeRTOS tasks are carefully prioritized and assigned appropriate stack sizes to ensure stability and responsiveness.