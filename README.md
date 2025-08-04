# 🚨 Accident and Rash Driving Alert System

This project is a smart embedded system designed to detect **accidents** and **rash driving behavior** using sensors and microcontrollers. It sends **real-time SMS alerts with location details** through GSM (SIM800L), utilizing **ESP-NOW** communication between ESP32 and ESP8266.

---

## 📌 Features

- 🚗 Accident detection via MPU6050 & vibration sensor  
- ⚠️ Rash driving detection using IR sensors  
- 📍 Real-time GPS tracking with Google Maps link  
- 📤 SMS alert system using SIM800L GSM module  
- 📡 Wireless communication using ESP-NOW  
- 🔴 LED indicators for visual feedback
 
---
 
## 🧰 Components Used

| Component           | Description                                |
|--------------------|--------------------------------------------|
| ESP32              | Main controller for accident detection     |
| ESP8266            | Receiver & GSM alert sender                |
| MPU6050            | Accelerometer + Gyroscope sensor           |
| NEO-6M             | GPS module                                 |
| SIM800L            | GSM module for sending SMS                 |
| IR Sensors (2x)    | Used for rash driving detection            |
| Vibration Sensor   | Detects sudden impact                      |
| LEDs               | Visual indicators (red/yellow)             |

---

## 🧠 System Architecture

The system is divided into two units:

### 1️⃣ Transmitter Unit (ESP32)
- Detects accident using MPU6050 and vibration sensor
- Gets GPS location from NEO-6M
- Sends location to ESP8266 using ESP-NOW

### 2️⃣ Receiver Unit (ESP8266)
- Receives data from ESP32
- Sends SMS alert with Google Maps link via SIM800L

---

Accident-RashDriving-Alert-System/
│
├── Accident_Code_ESP32/ # ESP32 transmitter code (accident)
├── Alert_Code_ESP8266/ # ESP8266 receiver code (alert sending)
├── Rash_Driving_Code/ # IR sensor-based rash driving detection
├── Circuits/ # Circuit diagrams (PDF/images)
├── Report/ # Final project report
├── Demo_Video/ # Project demo video (optional)
├── README.md # This file
├── LICENSE # Open-source license
└── .gitignore # Files to ignore in Git


---

## 🔍 How It Works

1. **Accident Detection**: MPU6050 + vibration sensor detect sudden shock or movement.
2. **GPS Tracking**: ESP32 captures location using NEO-6M GPS module.
3. **Wireless Communication**: ESP32 sends location via ESP-NOW to ESP8266.
4. **SMS Alert**: ESP8266 uses SIM800L to send an SMS with Google Maps link.
5. **Rash Driving**: IR sensors measure time difference to detect speeding behavior.

---

## 🚀 Getting Started

1. Upload `ESP32` code to ESP32 board.
2. Upload `ESP8266` code to ESP8266 board.
3. Upload `Rash Driving` code to relevant board (e.g., NodeMCU).
4. Connect components as per the provided circuit diagrams.
5. Power up, and monitor the serial console.

---

## 📸 Test Results

Screenshots of:
- SMS received with accident location
- Rash driving detection alerts  
_Available in the `Report/Test Results` section._

---

## 📜 License

This project is licensed under the **MIT License**.  
Feel free to use, modify, and distribute it. See the [LICENSE](LICENSE) file for details.

---






## 📁 Folder Structure

