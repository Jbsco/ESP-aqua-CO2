
# Aquarium CO₂ Control System

This project is a CO₂ control system for an aquarium, using the LilyGO T-QT V3 S3 (ESP32-S3) board. It maintains a target pH level by reading a pH sensor and controlling a CO₂ solenoid valve via a PID controller. The system features a real-time graph of pH values and solenoid status displayed on the onboard TFT.

---

## Features
- **pH Monitoring**: Reads pH values from a pH sensor.
- **CO₂ Control**: Uses a solenoid valve to regulate CO₂ injection based on a PID control system.
- **Real-Time Graphing**: Displays pH readings over time on the TFT display.
- **Status Indicators**: Shows current solenoid status (ON/OFF) and pH value.
- **User Adjustable**: Easily tune PID parameters, setpoint, and update intervals.

---

## Hardware Requirements
### Recommended Components
1. **Microcontroller**:
   - [LilyGO T-QT V3 S3 (ESP32-S3)](https://github.com/Xinyuan-LilyGO/T-QT)
2. **pH Sensor**:
   - [Atlas Scientific EZO-pH Kit](https://atlas-scientific.com/probes/ezo-ph-kit/) (High accuracy)
   - [DFRobot Gravity pH Sensor V2](https://www.dfrobot.com/product-1025.html) (Affordable, analog output)
3. **CO₂ Solenoid Valve**:
   - Any 12V solenoid valve (e.g., [AQUATEK CO₂ Check Valve](https://www.aquatek-california.com/)).
4. **Power Supply**:
   - 5V USB power supply for the LilyGO board.
   - 12V power supply for the solenoid valve.
5. **Other Components**:
   - Level shifter or MOSFET (e.g., IRLZ44N) for solenoid control.

---

## Wiring
| Component         | LilyGO Pin      | Notes                              |
|--------------------|-----------------|------------------------------------|
| pH Sensor Signal   | GPIO34 (ADC1_6) | Connect sensor output to ADC pin. |
| Solenoid Control   | GPIO25          | Use a MOSFET to drive the solenoid. |
| TFT Backlight      | GPIO2           | Controls display backlight.       |
| TFT Data/Clock     | GPIO19/GPIO18   | MOSI and SCLK for SPI.            |
| TFT Command        | GPIO15          | DC pin for TFT.                   |

> **Note**: Ensure the solenoid valve has a flyback diode if driven via a relay or MOSFET to protect the microcontroller.

---

## Software Setup
### 1. **Install PlatformIO**
- Install [PlatformIO IDE](https://platformio.org/) in your shell of choice, or use the VS Code extension.

### 2. **Clone This Repository**
```bash
git clone https://github.com/your-username/aquarium-co2-control
cd aquarium-co2-control
```

### 3. **Configure PlatformIO**
Ensure the `platformio.ini` file includes the following:
```ini
[env:lilygo_t_qt_s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
lib_deps = 
    br3ttb/PID@^1.2.1
    bodmer/TFT_eSPI@^2.5.0
```

### 4. **Compile and Upload**
Connect the board and run the following in PlatformIO:
```bash
pio run --target upload
```

---

## Adjusting Parameters
| Parameter       | Code Variable        | Default Value | Description                                  |
|------------------|----------------------|---------------|----------------------------------------------|
| Target pH        | `setpoint`           | `7.0`         | Desired pH level for the aquarium.           |
| PID Proportional | `kp`                 | `2.0`         | Adjust for responsiveness.                   |
| PID Integral     | `ki`                 | `0.5`         | Adjust for steady-state error correction.    |
| PID Derivative   | `kd`                 | `0.1`         | Adjust to reduce overshoot.                  |
| Update Interval  | `delay(500)`         | `500` ms      | Time between updates (ms).                   |

---

## Usage
1. Connect all hardware as per the wiring diagram.
2. Power on the system.
3. Monitor the real-time graph and solenoid status on the TFT display.
4. Use the serial monitor (baud: 115200) for debugging.

---

## Notes on Calibration
- Calibrate the pH sensor with standard buffer solutions (e.g., pH 4, 7, 10).
- Adjust the `map()` function in the code to match your sensor's voltage-to-pH range:
```cpp
input = map(rawADC, 0, ANALOG_READ_RESOLUTION, 4.0, 10.0);
```

---

## Future Enhancements
- Add button/touch controls for adjusting the setpoint dynamically.
- Implement Wi-Fi monitoring and control.
- Add data logging to an SD card or cloud service.

---

## References
1. [Atlas Scientific pH Sensor Setup](https://atlas-scientific.com/)
2. [DFRobot Gravity pH Sensor](https://www.dfrobot.com/product-1025.html)
3. [LilyGO T-QT V3 S3 Documentation](https://github.com/Xinyuan-LilyGO/T-QT)

---

Feel free to reach out with any questions or contributions! 😊