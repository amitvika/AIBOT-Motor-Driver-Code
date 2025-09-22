#include <Arduino.h>
#include "driver/twai.h" // The native ESP-IDF TWAI/CAN driver

// --- Pin Definitions ---
// CAN Transceiver Pins
const int CAN_STBY_PIN = 47; 
const gpio_num_t CAN_TX_PIN = GPIO_NUM_48;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_34;

// Motor Driver Pins
const int M1_FORWARD_PIN = 38;
const int M1_REVERSE_PIN = 39;
const int M2_FORWARD_PIN = 40; // CORRECTED PIN
const int M2_REVERSE_PIN = 37; // CORRECTED PIN

// --- PWM Configuration ---
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;

// --- CAN Configuration ---
const int MOTOR_CONTROL_CAN_ID = 0x100;

// --- Global Motor Speed Variables ---
float g_motor1_speed = 0.0;
float g_motor2_speed = 0.0;

//-- Timing for sending messages --//
const unsigned long sendInterval = 2000; // Send a message every 2 seconds
unsigned long previousMillis = 0;        // Stores the last time a message was sent

// --- Function Prototypes --- //
void setupCAN();
void receiveCAN();
void sendCAN();
void setMotorSpeed(int forward_pin, int reverse_pin, float speed);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("--- CAN Bus Motor Controller (Corrected Pins) ---");

  // Initialize CAN Bus Driver
  setupCAN();

  // Initialize Motor PWM Outputs
  ledcAttach(M1_FORWARD_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M1_REVERSE_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M2_FORWARD_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M2_REVERSE_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  Serial.println("Motor PWM outputs configured.");
}

void loop() {
  // 1. Check for incoming messages and update motor speeds if applicable
  receiveCAN();

  // 2. Apply the current speed values to the motors
  setMotorSpeed(M1_FORWARD_PIN, M1_REVERSE_PIN, g_motor1_speed);
  setMotorSpeed(M2_FORWARD_PIN, M2_REVERSE_PIN, g_motor2_speed);

  // 3. Send a message periodically (from your original script)
  sendCAN();
  
  delay(20); // Small delay to keep the loop stable
}

/**
 * @brief Configures, installs, and starts the ESP32's native TWAI/CAN driver.
 */
void setupCAN() {
  // Step 1: Enable the CAN transceiver chip
  pinMode(CAN_STBY_PIN, OUTPUT);
  digitalWrite(CAN_STBY_PIN, LOW); // Set to LOW for normal/active mode

  // Step 2: Configure the TWAI driver
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Step 3: Install and start the driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver");
    return;
  }
  Serial.println("CAN Driver started successfully!");
}

/**
 * @brief Checks for a received CAN message, prints it, and updates motor speeds.
 */
void receiveCAN() {
  twai_message_t message;

  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    // First, print any message that is received for debugging
    Serial.printf("RECEIVED << ID: 0x%lX | DLC: %d | Data: ", message.identifier, message.data_length_code);
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf("0x%02X ", message.data[i]);
    }
    Serial.println();

    // Next, check if this is a motor control command
    if (message.identifier == MOTOR_CONTROL_CAN_ID && message.data_length_code >= 2) {
      Serial.println("-> Motor command received. Updating speeds.");
      int8_t m1_raw_speed = (int8_t)message.data[0];
      int8_t m2_raw_speed = (int8_t)message.data[1];

      // Convert the -127 to 127 range to a -1.0 to 1.0 float
      g_motor1_speed = (float)m1_raw_speed / 127.0;
      g_motor2_speed = (float)m2_raw_speed / 127.0;
    }
  }
}

/**
 * @brief Sends a CAN message every 'sendInterval' milliseconds.
 */
void sendCAN() {
  if (millis() - previousMillis >= sendInterval) {
    previousMillis = millis(); // Update the timer
    twai_message_t message;
    message.identifier = 0x200;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = 4;
    message.data[0] = 0xAA;
    message.data[1] = 0xBB;
    message.data[2] = 0xCC;
    message.data[3] = 0xDD;

    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.printf("SENT >> ID: 0x%lX | Data: 0x%02X 0x%02X 0x%02X 0x%02X\n",
                    message.identifier, message.data[0], message.data[1], message.data[2], message.data[3]);
    } else {
      Serial.println("Failed to send message");
    }
  }
}

/**
 * @brief Sets the speed and direction for a motor.
 */
void setMotorSpeed(int forward_pin, int reverse_pin, float speed) {
  speed = constrain(speed, -1.0, 1.0);

  if (speed >= 0.0) {
    uint32_t dutyCycle = (uint32_t)(speed * 255.0);
    ledcWrite(forward_pin, dutyCycle);
    ledcWrite(reverse_pin, 0);
  } else {
    uint32_t dutyCycle = (uint32_t)(-speed * 255.0);
    ledcWrite(forward_pin, 0);
    ledcWrite(reverse_pin, dutyCycle);
  }
}