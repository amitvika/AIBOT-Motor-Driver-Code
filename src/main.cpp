#include <Arduino.h>
#include "driver/twai.h" // The native ESP-IDF TWAI/CAN driver

// --- Pin Definitions ---
const int CAN_STBY_PIN = 47; 
const gpio_num_t CAN_TX_PIN = GPIO_NUM_48;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_34;

const int M1_FORWARD_PIN = 38;
const int M1_REVERSE_PIN = 39;
const int M2_FORWARD_PIN = 40;
const int M2_REVERSE_PIN = 37;

// --- PWM Configuration ---
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;

// --- CAN Protocol Configuration (Matches vidd.py) ---
const uint32_t DRIVE_SYSTEM_ID = 0x200;
const uint8_t MOVE_COMMAND_BYTE = 0x01;

// --- Global Motor Speed Variables ---
float g_motor1_speed = 0.0;
float g_motor2_speed = 0.0;

// --- Function Prototypes ---
void setupCAN();
void receiveCAN();
void setMotorSpeed(int forward_pin, int reverse_pin, float speed);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("--- Drive System CAN Receiver (Differential Drive Logic) ---");

  setupCAN();

  ledcAttach(M1_FORWARD_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M1_REVERSE_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M2_FORWARD_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M2_REVERSE_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  Serial.println("Motor PWM outputs configured.");
}

void loop() {
  receiveCAN();
  setMotorSpeed(M1_FORWARD_PIN, M1_REVERSE_PIN, g_motor1_speed);
  setMotorSpeed(M2_FORWARD_PIN, M2_REVERSE_PIN, g_motor2_speed);
  delay(10);
}

void setupCAN() {
  pinMode(CAN_STBY_PIN, OUTPUT);
  digitalWrite(CAN_STBY_PIN, LOW);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = {
      .acceptance_code = (DRIVE_SYSTEM_ID << 21),
      .acceptance_mask = ~((uint32_t)TWAI_STD_ID_MASK << 21),
      .single_filter = true
  };

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver");
    return;
  }
  Serial.printf("CAN Driver started, listening for ID: 0x%lX\n", DRIVE_SYSTEM_ID);
}

/**
 * @brief Checks for a drive command CAN message and updates motor speeds using differential drive logic.
 */
void receiveCAN() {
  twai_message_t message;

  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (message.identifier == DRIVE_SYSTEM_ID) {
      if (message.data_length_code >= 5 && message.data[0] == MOVE_COMMAND_BYTE) {
        
        // --- NEW: DIFFERENTIAL DRIVE LOGIC ---

        // 1. Extract raw speed values (0-100)
        uint8_t f_speed = message.data[1];
        uint8_t b_speed = message.data[2];
        uint8_t l_speed = message.data[3];
        uint8_t r_speed = message.data[4];

        // 2. Calculate net throttle and steering values (-100 to 100)
        int net_throttle = f_speed - b_speed;
        int net_steering = r_speed - l_speed;

        // 3. Mix throttle and steering to get final speed for each motor
        int motor1_raw_speed = net_throttle + net_steering;
        int motor2_raw_speed = net_throttle - net_steering;

        // 4. Convert to float range (-1.0 to 1.0) and constrain the values
        // Note: We divide by 100.0 because that's our max input speed.
        g_motor1_speed = constrain((float)motor1_raw_speed / 100.0, -1.0, 1.0);
        g_motor2_speed = constrain((float)motor2_raw_speed / 100.0, -1.0, 1.0);
        
        Serial.printf("-> Drive RX | T:%d S:%d | M1: %.2f | M2: %.2f\n", 
                        net_throttle, net_steering, g_motor1_speed, g_motor2_speed);
      }
    }
  }
}

void setMotorSpeed(int forward_pin, int reverse_pin, float speed) {
  speed = constrain(speed, -1.0, 1.0);
  if (speed >= 0.0) {
    ledcWrite(forward_pin, (uint32_t)(speed * 255.0));
    ledcWrite(reverse_pin, 0);
  } else {
    ledcWrite(forward_pin, 0);
    ledcWrite(reverse_pin, (uint32_t)(-speed * 255.0));
  }
}