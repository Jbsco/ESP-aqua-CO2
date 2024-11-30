#include <Arduino.h>
#include <PID_v1.h>

#define PH_SENSOR_PIN 34  // ADC pin for pH sensor
#define SOLENOID_PIN 25   // GPIO pin to control CO2 solenoid
#define ANALOG_READ_RESOLUTION 4095.0  // 12-bit ADC

// PID parameters
double setpoint = 7.0;  // Target pH
double input, output;   // Input from pH sensor, output to solenoid control
double kp = 2.0, ki = 0.5, kd = 0.1; // PID coefficients

// Create PID instance
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
    Serial.begin(115200);
    pinMode(PH_SENSOR_PIN, INPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW); // Ensure solenoid starts closed

    // Initialize PID controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 1); // Output range for solenoid (0 = OFF, 1 = ON)

    Serial.println("CO2 Control System Initialized!");
}

void loop() {
    // Read and process pH sensor value (simulate mapping voltage to pH)
    int rawADC = analogRead(PH_SENSOR_PIN);
    input = map(rawADC, 0, ANALOG_READ_RESOLUTION, 4.0, 10.0); // Simulated pH range (adjust as needed)

    // Run PID computation
    myPID.Compute();

    // Control solenoid
    if (output > 0.5) {
        digitalWrite(SOLENOID_PIN, HIGH); // Open solenoid
        Serial.println("CO2 Solenoid: OPEN");
    } else {
        digitalWrite(SOLENOID_PIN, LOW); // Close solenoid
        Serial.println("CO2 Solenoid: CLOSED");
    }

    // Debugging output
    Serial.print("Raw ADC: ");
    Serial.print(rawADC);
    Serial.print(" | pH: ");
    Serial.print(input, 2);
    Serial.print(" | PID Output: ");
    Serial.println(output, 2);

    delay(1000); // 1-second control interval
}
