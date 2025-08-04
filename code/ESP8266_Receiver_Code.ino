#include <ESP8266WiFi.h>     // Required for ESP8266 Wi-Fi operations
#include <espnow.h>          // ESP-NOW protocol for peer-to-peer communication
#include <SoftwareSerial.h>  // To communicate with SIM800L over software-defined pins

// Create software serial instance for SIM800L (RX=GPIO13, TX=GPIO15)
SoftwareSerial sim800(13, 15);

// String to store the incoming Google Maps link
String receivedLink = "";

// Flag to indicate when to send SMS
bool sendSMSFlag = false;

// -----------------------------------------
// 🕒 Safe delay to avoid WDT (Watchdog Timer) reset
// -----------------------------------------
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    delay(1);  // Light CPU wait
  }
}

// -----------------------------------------
// 📩 Callback function triggered when ESP-NOW data is received
// Converts raw bytes into a String (Google Maps link)
// -----------------------------------------
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  receivedLink = "";  // Reset string
  for (int i = 0; i < len; i++) {
    receivedLink += (char)incomingData[i];  // Convert bytes to characters
  }

  Serial.println("📥 Message Received via ESP-NOW:");
  Serial.println(receivedLink);

  sendSMSFlag = true;  // Set flag to send SMS in main loop
}

// -----------------------------------------
// 📤 Function to send SMS via SIM800L with the GPS location
// -----------------------------------------
void sendSMS(String link) {
  Serial.println("📤 Sending SMS...");

  // Initialize SIM800L communication
  sim800.println("AT");
  safeDelay(1000);

  // Set SIM800L to text mode
  sim800.println("AT+CMGF=1");
  safeDelay(1000);

  // Replace the number with the desired recipient
  sim800.println("AT+CMGS=\"+919008383803\"");
  safeDelay(1000);

  // Construct and send the message
  sim800.print("Accident Location: ");
  sim800.print("https://maps.google.com/?q=");
  sim800.print(link);
  sim800.write(26);  // Ctrl+Z character to send the SMS
  safeDelay(5000);   // Wait for sending to complete

  Serial.println("✅ SMS Sent.");
}

// -----------------------------------------
// 🚀 Setup function: Initializes serial, ESP-NOW, and SIM800L
// -----------------------------------------
void setup() {
  Serial.begin(9600);         // Debug serial monitor
  sim800.begin(9600);         // Start SIM800L communication
  delay(3000);                // Allow SIM800L to initialize

  // Set ESP8266 to Station mode (needed for ESP-NOW)
  WiFi.mode(WIFI_STA);
  Serial.println("🚦 ESP8266 Receiver + SIM800L");

  // Initialize ESP-NOW protocol
  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed");
    return;  // Exit if failed
  }

  // Set device role and register callback for incoming data
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("✅ ESP-NOW Ready");
}

// -----------------------------------------
// 🔁 Loop function: Sends SMS when data is received
// -----------------------------------------
void loop() {
  if (sendSMSFlag) {
    sendSMSFlag = false;           // Reset flag to prevent multiple messages
    sendSMS(receivedLink);         // Call function to send SMS
  }
}
