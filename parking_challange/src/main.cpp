#include <esp_now.h>
#include <WiFi.h>
#include <esp_sleep.h>  // For deep sleep functions

#define TRIG_PIN 5   // ESP32-DevKitC V4 recommended GPIO for Trigger
#define ECHO_PIN 18  // ESP32-DevKitC V4 recommended GPIO for Echo
#define POT_PIN 34   // Pin connected to the potentiometer

uint8_t receiverAddress[] = {0x8C, 0xAA, 0xB5, 0x84, 0xFB, 0x90};  // FIXED SYNTAX ERROR

esp_now_peer_info_t peerConfig;

#define MICRO_TO_SEC 1000000
#define SLEEP_INTERVAL 37  // Sleep duration in seconds

RTC_DATA_ATTR int bootCounter = 0;  // Persistent variable across deep sleep

// Power consumption values (in mW) from CSV files
#define POWER_DEEP_SLEEP 59.89   // Measured power during deep sleep
#define POWER_IDLE 320.91        // Estimated idle power
#define POWER_SENSOR_READ 467.11 // Measured power during sensor reading
#define POWER_WIFI 704.75        // Measured power during WiFi transmission
#define POWER_TX_2DBM 797.29     // Assumed power for 2dBm transmission
#define POWER_TX_19_5DBM 1221.76 // Assumed power for 19.5dBm transmission

// Callback for sending status
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Message Delivery Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Callback for receiving data
void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.print("Received: ");
    char buffer[len + 1];
    memcpy(buffer, incomingData, len);
    buffer[len] = '\0';  // Null terminate the string
    Serial.println(String(buffer));
}

// Function to estimate power consumption for each state
void calculateAndPrintPowerConsumption() {
    Serial.println("-------------------------------------------------");
    Serial.println("Average power consumption in deep sleep: " + String(POWER_DEEP_SLEEP, 2) + " mW");
    Serial.println("Average power consumption in idle: " + String(POWER_IDLE, 2) + " mW");
    Serial.println("Average power consumption in sensor reading: " + String(POWER_SENSOR_READ, 2) + " mW");
    Serial.println("Average power consumption in wifi: " + String(POWER_WIFI, 2) + " mW");
    Serial.println("Average power consumption in transmission at 2dBm: " + String(POWER_TX_2DBM, 2) + " mW");
    Serial.println("Average power consumption in transmission at 19.5dBm: " + String(POWER_TX_19_5DBM, 2) + " mW");
    Serial.println("-------------------------------------------------");
}

// Function to estimate energy consumption per transmission cycle
float estimateTransmissionCycleEnergy() {
    // Time spent in each state (seconds)
    float time_deep_sleep = SLEEP_INTERVAL - 1;  // Most time spent in deep sleep
    float time_idle = 0.3;
    float time_sensor_read = 0.2;
    float time_wifi = 0.2;
    float time_transmission = 0.1; // Transmission time at 2dBm

    // Compute energy in Joules (E = P × t / 1000 to convert mW to W)
    float energy_deep_sleep = (POWER_DEEP_SLEEP * time_deep_sleep) / 1000;
    float energy_idle = (POWER_IDLE * time_idle) / 1000;
    float energy_sensor_read = (POWER_SENSOR_READ * time_sensor_read) / 1000;
    float energy_wifi = (POWER_WIFI * time_wifi) / 1000;
    float energy_transmission = (POWER_TX_2DBM * time_transmission) / 1000; // Assuming 2dBm transmission power

    // Total energy per cycle
    float total_energy = energy_deep_sleep + energy_idle + energy_sensor_read + energy_wifi + energy_transmission;

    // Print detailed energy breakdown
    Serial.println("-------------------------------------------------");
    Serial.println("Energy Consumption Breakdown (1 Transmission Cycle):");
    Serial.println("Deep Sleep: " + String(energy_deep_sleep, 6) + " J");
    Serial.println("Idle/Wakeup: " + String(energy_idle, 6) + " J");
    Serial.println("Sensor Reading: " + String(energy_sensor_read, 6) + " J");
    Serial.println("WiFi Activation: " + String(energy_wifi, 6) + " J");
    Serial.println("Transmission: " + String(energy_transmission, 6) + " J");
    Serial.println("-------------------------------------------------");
    Serial.println("Total Energy Consumption per Cycle: " + String(total_energy, 6) + " J");
    Serial.println("-------------------------------------------------");

    return total_energy;
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    bootCounter++;

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    // Configure peer connection
    memset(&peerConfig, 0, sizeof(peerConfig)); // Ensure peerConfig is initialized properly
    memcpy(peerConfig.peer_addr, receiverAddress, 6);
    peerConfig.channel = 0;
    peerConfig.encrypt = false;

    if (esp_now_add_peer(&peerConfig) != ESP_OK) { // Ensure the peer is added successfully
        Serial.println("Failed to add ESP-NOW peer");
        return;
    }

    // Works independently for each parking spot
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(POT_PIN, INPUT);

    // Trigger ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure distance
    int pulseDuration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
    int measuredDistance = (pulseDuration > 0) ? (pulseDuration / 58) : -1;  // -1 timeout
    Serial.println("Measured Distance: " + String(measuredDistance) + "cm");

    // Parking status between 0 and 50 = OCCUPIED, otherwise = FREE
    String parkingStatus = (measuredDistance > 0 && measuredDistance < 50) ? "OCCUPIED" : "FREE";

    // Read potentiometer value (0-4095)
    int potValue = analogRead(POT_PIN);
    float voltage = (potValue / 4095.0) * 3.3; // Map ADC to 3.3V range
    Serial.println("Potentiometer Value: " + String(potValue) + " (" + String(voltage, 2) + "V)");

    // Print calculated power consumption for each state
    calculateAndPrintPowerConsumption();

    // Estimate and print total energy consumption per cycle
    estimateTransmissionCycleEnergy();

    // Combine parking status and potentiometer value
    String message = parkingStatus + " | Pot: " + String(potValue);

    // Transmit status
    esp_now_send(receiverAddress, (uint8_t *)message.c_str(), message.length() + 1);

    // Output is printed before deep sleep to fix flash boot issues
    delay(2000);

    // Deep sleep for X = 37 seconds
    esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL * MICRO_TO_SEC);
    Serial.flush();
    esp_deep_sleep_start();
}

void loop() {
    // We can leave the loop empty because we are using deep sleep
}
