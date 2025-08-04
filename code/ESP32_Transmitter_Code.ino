#include <Wire.h>                      // For I2C communication (used by MPU6050)
#include <TinyGPS++.h>                // GPS parsing library
#include <Adafruit_MPU6050.h>         // Library for MPU6050 sensor
#include <Adafruit_Sensor.h>          // Adafruit unified sensor library
#include <HardwareSerial.h>           // For using multiple serial ports
#include <esp_now.h>                  // For ESP-NOW communication
#include <WiFi.h>                     // Required for ESP-NOW setup

// MAC Address of the receiver ESP8266 (replace with actual MAC if different)
uint8_t receiverMacAddress[] = {0x8C, 0xAA, 0xB5, 0xD3, 0xA5, 0x5A};

// Pin and threshold definitions
#define VIBRATION_PIN 4               // Digital pin connected to the vibration sensor
#define ACCEL_THRESHOLD 3.0           // Threshold to detect sudden acceleration
#define GPS_TIMEOUT 300000            // Timeout for GPS fix (5 minutes)
#define UPDATE_INTERVAL 5000          // Delay between GPS sends (after trigger)
#define DIAG_INTERVAL 10000           // Interval for printing GPS diagnostics
#define DEBUG_NMEA true               // Toggle to print raw NMEA data

// Create instances for sensors and serial ports
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);         // Use UART2: RX=16, TX=17

const int statusLed = 2;              // Optional status LED (onboard GPIO2)
bool gpsReady = false;
unsigned long lastUpdateTime = 0;
unsigned long lastDiagTime = 0;
bool initialFixAttempt = true;

void setup() {
  Serial.begin(115200);               // Serial monitor for debugging
  
  // Setup status LED
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, LOW);
  
  // Setup vibration sensor pin
  pinMode(VIBRATION_PIN, INPUT);
  
  // Initialize MPU6050 sensor over I2C (SDA=21, SCL=22)
  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not detected. Check wiring (SDA->GPIO21, SCL->GPIO22, VCC->3.3V, GND->GND)");
    while (1) delay(10);             // Halt execution if MPU not found
  }
  // Configure MPU6050 sensitivity settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("✅ MPU6050 initialized");

  // Start GPS module over UART2
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("ESP32 - GPS, MPU6050, Vibration Sensor Test");
  Serial.println("Waiting for GPS signal... Ensure antenna has clear sky view.");
  Serial.println("Location: Terrace (open sky)");

  // Initialize ESP-NOW communication
  WiFi.mode(WIFI_STA);               // Set ESP32 to station mode
  Serial.println("Wi-Fi MAC Address: " + WiFi.macAddress());
  delay(100);                        // Wait for Wi-Fi to settle
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed");
    while (1) delay(10);             // Halt if ESP-NOW fails
  }

  // Register peer (ESP8266)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    while (1) delay(10);
  }
  Serial.println("✅ ESP-NOW initialized");
}

void loop() {
  // Process incoming GPS data
  unsigned long charsProcessed = 0;
  while (GPS_Serial.available() && charsProcessed < 1000) {
    char c = GPS_Serial.read();
    if (gps.encode(c)) {
      lastUpdateTime = millis();     // Track when data was last updated
    }
    if (DEBUG_NMEA && !gpsReady) {
      Serial.write(c);               // Print raw GPS data
    }
    charsProcessed++;
  }

  // Handle GPS timeout error
  if (millis() - lastUpdateTime > GPS_TIMEOUT && gps.charsProcessed() < 10) {
    Serial.println("ERROR: No GPS data received. Check:");
    Serial.println("- Wiring: NEO-6M TX->GPIO16 (RX), RX->GPIO17 (TX), VCC->3.3V, GND->GND");
    Serial.println("- Antenna: Ensure connected and upright with clear sky view");
    Serial.println("- Power: Stable 3.3V supply");
    Serial.println("- Baud rate: Set to 9600");
    digitalWrite(statusLed, LOW);
    delay(5000);
    return;
  }

  // Check if GPS has valid fix
  if (!gpsReady && gps.location.isValid() && gps.satellites.isValid() && gps.satellites.value() >= 4) {
    gpsReady = true;
    digitalWrite(statusLed, HIGH);
    Serial.println("✅ GPS Lock Acquired!");
    Serial.print("Initial Location: https://maps.google.com/?q=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.println(gps.location.lng(), 6);
    initialFixAttempt = false;       // Stop raw NMEA printing
  }

  // Print periodic GPS diagnostic info
  if (millis() - lastDiagTime > DIAG_INTERVAL) {
    Serial.print("Satellites in view: ");
    Serial.println(gps.satellites.isValid() ? gps.satellites.value() : "UNKNOWN");
    Serial.print("HDOP (Accuracy): ");
    Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : "UNKNOWN");
    lastDiagTime = millis();
  }

  // Check if vibration sensor is triggered
  bool triggered = false;
  if (digitalRead(VIBRATION_PIN) == HIGH) {
    Serial.println("⚠️ Vibration Detected!");
    triggered = true;
  }

  // Check for abnormal motion using MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float totalAccel = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );
  if (abs(totalAccel - 9.8) > ACCEL_THRESHOLD) {
    Serial.println("⚠️ Sudden Motion Detected by MPU6050!");
    triggered = true;
  }

  // If accident is detected, send location via ESP-NOW
  if (triggered) {
    Serial.println("📍 Getting location...");
    if (gps.location.isValid()) {
      digitalWrite(statusLed, HIGH);
      
      // Format GPS location as Google Maps link
      char message[100];
      snprintf(message, sizeof(message), "https://maps.google.com/?q=%.6f,%.6f", 
               gps.location.lat(), gps.location.lng());

      // Display in serial monitor
      Serial.println("📍 Triggered Location:");
      Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
      Serial.println("Google Maps Link: " + String(message));

      // Send location to ESP8266 via ESP-NOW
      esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)message, strlen(message) + 1);
      if (result == ESP_OK) {
        Serial.println("✅ Location sent via ESP-NOW");
      } else {
        Serial.println("❌ ESP-NOW send failed");
      }
    } else {
      Serial.println("❌ No valid GPS location available.");
      digitalWrite(statusLed, LOW);
    }
    delay(UPDATE_INTERVAL);          // Debounce time between triggers
  }

  // Handle GPS fix lost condition
  if (gpsReady && !gps.location.isValid() && gps.charsProcessed() > 10 && millis() - lastUpdateTime > 1000) {
    Serial.println("Waiting for GPS fix... Ensure clear sky view.");
    Serial.print("Satellites in view: ");
    Serial.println(gps.satellites.isValid() ? gps.satellites.value() : "UNKNOWN");
    Serial.print("HDOP (Accuracy): ");
    Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : "UNKNOWN");
    digitalWrite(statusLed, LOW);
    gpsReady = false;
    initialFixAttempt = true;        // Resume printing raw NMEA
  }

  delay(200);                        // Main loop delay
}
