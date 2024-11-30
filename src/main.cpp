#include <Arduino.h>
#include <PID_v1.h>
#include <TFT_eSPI.h>

#define PH_SENSOR_PIN 16  // ADC pin for pH sensor
#define SOLENOID_PIN 21   // GPIO pin to control CO2 solenoid
#define ANALOG_READ_RESOLUTION 4095.0  // 12-bit ADC

// PID parameters
double setpoint = 8.9;  // Target pH
double input, output;   // Input from pH sensor, output to solenoid control
double kp = 2.0, ki = 0.5, kd = 0.1; // PID coefficients

// Create PID instance
PID myPID(&input, &output, &setpoint, kp, ki, kd, REVERSE);

// TFT display
TFT_eSPI tft = TFT_eSPI();
#define GRAPH_HEIGHT 100
#define GRAPH_WIDTH 320
uint16_t graphData[GRAPH_WIDTH]; // Array to store pH values for plotting

void setup() {
    Serial.begin(115200);
    pinMode(PH_SENSOR_PIN, INPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW); // Ensure solenoid starts closed

    // Initialize PID controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 1); // Output range for solenoid (0 = OFF, 1 = ON)

    // Initialize TFT
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);

    // Clear graph data
    memset(graphData, 0, sizeof(graphData));

    Serial.println("CO2 Control System Initialized!");
}

void loop() {
    // Read and process pH sensor value (simulate mapping voltage to pH)
    int rawADC = analogRead(PH_SENSOR_PIN);
    input = map(rawADC, 0, ANALOG_READ_RESOLUTION, 4.0, 10.0); // Simulated pH range (adjust as needed)

    // Run PID computation
    myPID.Compute();

    // Control solenoid
    bool solenoidState = output > 0.5 && input > setpoint; // Turn ON if output > 0.5 and pH is above setpoint
    digitalWrite(SOLENOID_PIN, solenoidState ? HIGH : LOW);

    // Update graph
    memmove(graphData, graphData + 1, (GRAPH_WIDTH - 1) * sizeof(uint16_t));
    graphData[GRAPH_WIDTH - 1] = GRAPH_HEIGHT - map(input * 10, 40, 100, 0, GRAPH_HEIGHT);

    // Clear and redraw graph area
    tft.fillRect(0, 0, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);
    for (int i = 1; i < GRAPH_WIDTH; i++) {
        tft.drawLine(i - 1, graphData[i - 1], i, graphData[i], TFT_GREEN);
    }

    // Display solenoid status
    tft.setTextSize(2);
    tft.setCursor(0, GRAPH_HEIGHT + 10);
    tft.setTextColor(solenoidState ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(solenoidState ? "CO2 Solenoid: ON " : "CO2 Solenoid: OFF");

    // Display pH value
    tft.setCursor(0, GRAPH_HEIGHT + 40);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Current pH: %.2f", input);

    // Debug output
    Serial.printf("Raw ADC: %d | pH: %.2f | PID out: %.2f | Solenoid: %s\n", rawADC, input, output, solenoidState ? "ON" : "OFF");

    delay(500); // Update interval
}
