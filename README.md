
# Aquarium CO₂ Control System

This project is a CO₂ control system for an automated aquarium, using the LilyGO T-Display-S3 ESP32 board. It maintains a target pH level by reading a pH sensor and controlling a CO₂ solenoid valve via a PID controller. The system features a real-time graph of pH values and solenoid status displayed on the onboard TFT.

**Note: Using pH to determine whether your tank needs CO₂ is not a very accurate method unless all other parameters are stable, and even then frequent calibration may be necessary. Measuring dissolved CO₂ directly requires costly sensors. Please understand that many other parameters affect pH. This means that the default values in this project may not be appropriate for your setup. This is not a guaranteed solution for CO₂ automation and will require an in-depth understanding of your own tank environment and its water parameters.**

**Another Note: I am slowly setting up a tank in my spare time, and it will take me a while to test this thoroughly. This is a pretty low-priority project. Please do not use this in your tank with live flora/fauna unless you have tested the functionality in a dry scenario and determined it to be sufficient for your use case. I'm providing this repository to improve accessibility to lower cost CO₂ automation solutions and to version control any improvements on the concept. Forks are welcome. If you have comments, feedback, or requests, please feel free to create an issue or open a discussion thread.**

---

## Features
- **pH Monitoring**: Reads pH values from a pH sensor using UART, I²C, or analog protocols.
- **CO₂ Control**: Uses a solenoid valve to regulate CO₂ injection based on a PID control system.
- **Real-Time Graphing**: Displays pH readings over time on the TFT display.
- **Status Indicators**: Shows current solenoid status (ON/OFF) and pH value.
- **User Adjustable**: Easily tune PID parameters, setpoint, and update intervals.

---

## Hardware Requirements
### Recommended Components
1. **Microcontroller**:
   - [LilyGO T-Display-S3 (ESP32-S3)](https://github.com/Xinyuan-LilyGO/T-Display-S3)
2. **pH Sensor**:
   - [Atlas Scientific EZO-pH Kit](https://atlas-scientific.com/kits/ph-kit/) (Higher accuracy)
   - [DFRobot Gravity pH Sensor V2](https://www.dfrobot.com/product-1025.html) (Affordable, analog output)
3. **CO₂ Solenoid Valve**:
   - Any 12V solenoid valve (e.g., [AQUATEK CO₂ Solenoid Valve](https://www.aquatek-california.com/)).
4. **Power Supply**:
   - 5V USB power supply for the LilyGO board.
   - 12V power supply for the solenoid valve.
5. **Other Components**:
   - Level shifter or MOSFET (e.g., IRLZ44N) for solenoid control.

---

## Wiring
| Component         | LilyGO Pin      | Notes                              |
|--------------------|-----------------|------------------------------------|
| pH Sensor Signal (Analog)   | GPIO18 (ADC2_7) | Connect sensor output to ADC pin. |
| pH Sensor Signal (UART)   | GPIO18 (U1 RXD) | Connect sensor RX to RX pin. |
| pH Sensor Signal (UART)   | GPIO17 (U1 TXD) | Connect sensor TX to TX pin. |
| pH Sensor Signal (I²C)   | GPIO44 (SCL) | Connect sensor SCL to SCL pin. |
| pH Sensor Signal (I²C)   | GPIO43 (SDA) | Connect sensor SDA to SDA pin. |
| Solenoid Control   | GPIO21          | Use a MOSFET to drive the solenoid. |

> **Note**: Ensure the solenoid valve has a flyback diode if driven via a relay or MOSFET to protect the microcontroller.

---

## Software Setup
### 1. **Install PlatformIO**
- Install [PlatformIO](https://platformio.org/) in your shell of choice, or use the VS Code extension.

### 2. **Clone This Repository**
```bash
git clone https://github.com/Jbsco/ESP-aqua-CO2
cd ESP-aqua-CO2
```

### 3. **Set the Sensor Mode**
- In `src/main.cpp` edit line #12 to use UART, I²C, or analog sensor input:
```
// INPUT MODES:
#define PH_SENSOR_MODE_UART   1 // UART with Atlas Scientific EZO-pH
#define PH_SENSOR_MODE_I2C    2 // I2C with Atlas Scientific EZO-pH
#define PH_SENSOR_MODE_ANALOG 3 // Analog with DFRobot pH sensor

// INPUT SELECTION:
#define PH_SENSOR_MODE PH_SENSOR_MODE_ANALOG  // Set to desired mode
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
1. Connect all hardware as per the wiring table.
2. Power on the system.
3. Monitor the real-time graph and solenoid status on the TFT display.
4. Use the serial monitor (baud: 115200) for debugging.
5. Adjust parameters as needed.

---

## Notes on Analog Calibration
- Calibrate the pH sensor with standard buffer solutions (e.g., pH 4, 7, 10).
- Adjust the `map()` function in the code to match your sensor's voltage-to-pH range:
```cpp
input = map(rawADC, 0, ANALOG_READ_RESOLUTION, 4.0, 10.0);
```

---

## Future Enhancements
- Improve TFT display interface to show graph markers, setpoint, PID reponse, & any other parameters deemed useful.
- Add photos/graphics of device wiring, setup examples, & results to README.
- Add button control for adjusting the setpoint.
- Implement Wi-Fi monitoring and control.
- Add data logging to an SD card or cloud service.
- Extend functionality to lighting schedule/intensity/RGB control.
- Extend to any other possible control outputs.

---

## References
1. [Atlas Scientific pH Sensor Setup](https://atlas-scientific.com/)
2. [DFRobot Gravity pH Sensor](https://www.dfrobot.com/product-1025.html)
3. [LilyGO T-Display-S3 Documentation](https://github.com/Xinyuan-LilyGO/T-Display-S3)
