#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Define motor control pins for two L9110 motor drivers
const int MOTOR1_A = 25;
const int MOTOR1_B = 26;
const int MOTOR2_A = 27;
const int MOTOR2_B = 21;
const int MOTOR3_A = 16;
const int MOTOR3_B = 13;
const int MOTOR4_A = 33;
const int MOTOR4_B = 32;

// Define LED pin for connection status
const int STATUS_LED_PIN = 2;

// Define PWM range
const int PWM_RANGE = 255;

// Joystick data structure
typedef struct {
    int16_t carX;
    int16_t carY;
    
    bool carButton;
    
} JoystickData;

// Create a variable to store the received joystick data
JoystickData joystickData;

// Track connection status
bool isConnected = false;
unsigned long lastDataTime = 0;

// Function to control motor speeds
void controlMotors(int x, int y) {
    int leftSpeed = y + x;
    int rightSpeed = y - x;

    leftSpeed = constrain(leftSpeed, -PWM_RANGE, PWM_RANGE);
    rightSpeed = constrain(rightSpeed, -PWM_RANGE, PWM_RANGE);

    Serial.printf("Motor Speeds: Left=%d, Right=%d\n", leftSpeed, rightSpeed);

    if (leftSpeed > 0) {
        analogWrite(MOTOR1_A, leftSpeed);
        analogWrite(MOTOR1_B, 0);
    } else {
        analogWrite(MOTOR1_A, 0);
        analogWrite(MOTOR1_B, -leftSpeed);
    }

    if (rightSpeed > 0) {
        analogWrite(MOTOR2_A, rightSpeed);
        analogWrite(MOTOR2_B, 0);
    } else {
        analogWrite(MOTOR2_A, 0);
        analogWrite(MOTOR2_B, -rightSpeed);
    }
}

// Function to update connection status
void updateConnectionStatus() {
    unsigned long currentTime = millis();
    if (currentTime - lastDataTime > 2000) {
        if (isConnected) {
            isConnected = false;
            Serial.println("Connection lost. No data received for 2 seconds.");
            digitalWrite(STATUS_LED_PIN, LOW);
        }
    } else if (!isConnected) {
        isConnected = true;
        Serial.println("Connection established. Receiving data...");
        digitalWrite(STATUS_LED_PIN, HIGH);
    }
}

// ESP-NOW data receive callback
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(JoystickData)) {
        memcpy(&joystickData, data, len);

        // Update the time of the last received data
        lastDataTime = millis();

        // Print received data
        Serial.println("\nReceived Joystick Data:");
        Serial.printf("  Car X-axis: %d\n", joystickData.carX);
        Serial.printf("  Car Y-axis: %d\n", joystickData.carY);
        
        Serial.printf("  Car Button: %s\n", joystickData.carButton ? "Pressed" : "Released");
       

        // Control the motors based on received data
        Serial.println("Processing motor control...");
        controlMotors(joystickData.carX, joystickData.carY);
    } else {
        Serial.println("Received data size mismatch. Ignoring data.");
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize motor pins
    pinMode(MOTOR1_A, OUTPUT);
    pinMode(MOTOR1_B, OUTPUT);
    pinMode(MOTOR2_A, OUTPUT);
    pinMode(MOTOR2_B, OUTPUT);
    pinMode(MOTOR3_A, OUTPUT);
    pinMode(MOTOR3_B, OUTPUT);
    pinMode(MOTOR4_A, OUTPUT);
    pinMode(MOTOR4_B, OUTPUT);

    // Set motor pins to LOW
    digitalWrite(MOTOR1_A, LOW);
    digitalWrite(MOTOR1_B, LOW);
    digitalWrite(MOTOR2_A, LOW);
    digitalWrite(MOTOR2_B, LOW);
    digitalWrite(MOTOR3_A, LOW);
    digitalWrite(MOTOR3_B, LOW);
    digitalWrite(MOTOR4_A, LOW);
    digitalWrite(MOTOR4_B, LOW);

    // Initialize the status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // Initialize Wi-Fi in station mode
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register receive callback
    esp_now_register_recv_cb(onDataReceived);

    Serial.println("Receiver ready. Waiting for data...");
}

void loop() {
    // Update connection status based on the last received data
    updateConnectionStatus();
}
