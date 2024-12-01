#include <Arduino.h>
#include <PID_v1.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#define SERIAL_DEBUG_OUTPUT 0 // Enable/disable serial debug output

//INPUT MODES:
#define PH_SENSOR_MODE_UART   1 // UART with Atlas Scientific EZO-pH
#define PH_SENSOR_MODE_I2C    2 // I2C with Atlas Scientific EZO-pH
#define PH_SENSOR_MODE_ANALOG 3 // Analog with DFRobot pH sensor

// INPUT SELECTION:
#define PH_SENSOR_MODE PH_SENSOR_MODE_ANALOG // Set to desired mode

// UART
#if PH_SENSOR_MODE == PH_SENSOR_MODE_UART
    HardwareSerial ezoSerial(2);
    #define TX_PIN 17
    #define RX_PIN 18
    String response;
// I2C
#elif PH_SENSOR_MODE == PH_SENSOR_MODE_I2C
    #define EZO_I2C_ADDRESS 0x63
    #define SDA_PIN 43
    #define SCL_PIN 44
    String response;
// Analog
#elif PH_SENSOR_MODE == PH_SENSOR_MODE_ANALOG
    #define PH_SENSOR_PIN 18 // ADC pin for pH sensor
    double response;
#endif

#define SOLENOID_PIN 21 // GPIO pin to control CO2 solenoid

// PID parameters
double setpoint = 7.0; // Target pH
double input, output; // Input from pH sensor, output to solenoid control
double kp = 25.0, ki = 1.0, kd = 5.0; // PID coefficients

// Create PID instance
PID myPID(&input, &output, &setpoint, kp, ki, kd, REVERSE);

// TFT display
TFT_eSPI tft = TFT_eSPI();
#define GRAPH_HEIGHT 100
#define GRAPH_WIDTH 320
double graphData[GRAPH_WIDTH]; // Array to store pH values for plotting

void setup() {
    Serial.begin(115200);

    // INPUT INITIALIZATION
    // UART
    #if PH_SENSOR_MODE == PH_SENSOR_MODE_UART
        ezoSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    // I2C
    #elif PH_SENSOR_MODE == PH_SENSOR_MODE_I2C
        Wire.begin(SDA_PIN, SCL_PIN);
    // Analog
    #elif PH_SENSOR_MODE == PH_SENSOR_MODE_ANALOG
        pinMode(PH_SENSOR_PIN, INPUT);
    #endif

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
    // UART
    #if PH_SENSOR_MODE == PH_SENSOR_MODE_UART
        // Request pH from EZO-pH via UART
        ezoSerial.print("R\r"); // Send read command
        delay(1000); // Wait for response
        if (ezoSerial.available()) {
            response = ezoSerial.readStringUntil('\r');
            input = response.toFloat();
        }
    // I2C
    #elif PH_SENSOR_MODE == PH_SENSOR_MODE_I2C
        // Request pH from EZO-pH via I2C
        Wire.beginTransmission(EZO_I2C_ADDRESS);
        Wire.write('R'); // Send read command
        Wire.endTransmission();
        delay(1000); // Wait for response
        Wire.requestFrom(EZO_I2C_ADDRESS, 7); // Request response (7 bytes max)
        if (Wire.available() >= 4) {
            response = "";
            while (Wire.available())
                response += (char)Wire.read();
            input = response.toFloat();
        }
    // Analog
    #elif PH_SENSOR_MODE == PH_SENSOR_MODE_ANALOG
        response = analogRead(PH_SENSOR_PIN);
        // Map to pH range with finer resolution
        double voltage = response / 4095.0 * 3.3; // Calculate voltage
        input = 10.0 - (voltage / 3.3 * (10.0 - 4.0)); // Map to simulated pH range (adjust as needed)
    #endif

    // Run PID computation
    myPID.Compute();

    // Control solenoid
    bool solenoidState = output > 0.5; // Turn ON if output > 0.5 and pH is above setpoint
    digitalWrite(SOLENOID_PIN, solenoidState ? HIGH : LOW);

    // Update graph data
    memmove(graphData, graphData + 1, (GRAPH_WIDTH - 1) * sizeof(double));
    graphData[GRAPH_WIDTH - 1] = GRAPH_HEIGHT - map(input * 10, 40, 100, 0, GRAPH_HEIGHT);
    // Clear and redraw graph area
    tft.fillRect(0, 0, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);
    // Draw dashed red line for setpoint
    double setpointY = GRAPH_HEIGHT - map(setpoint * 10, 40, 100, 0, GRAPH_HEIGHT); // Map setpoint to graph Y range
    for (int x = 10; x < 310; x += 10) // Dashed pattern: line every 10px
        tft.drawLine(x, setpointY, x + 5, setpointY, TFT_RED); // Short dashes
    // Draw data
    for (int i = 1; i < GRAPH_WIDTH; i++)
        tft.drawPixel(i - 1, graphData[i], graphData[i] > setpointY ? TFT_GREEN : TFT_YELLOW);

    // Display solenoid status
    tft.setTextSize(2);
    tft.setCursor(0, GRAPH_HEIGHT + 10);
    tft.setTextColor(solenoidState ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(solenoidState ? "CO2 Solenoid: ON " : "CO2 Solenoid: OFF");

    // Display pH value
    tft.setCursor(0, GRAPH_HEIGHT + 40);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Current pH: %.2f", input);

    #if SERIAL_DEBUG_OUTPUT
        // Debug output
        Serial.printf("Response: %.2f | pH: %.2f | PID out: %.2f | Solenoid: %s\n", response, input, output, solenoidState ? "ON" : "OFF");
    #endif

    delay(100); // Update interval
}
