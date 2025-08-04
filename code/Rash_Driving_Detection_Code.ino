#include <SoftwareSerial.h>  // For serial communication with SIM800L module

// -------------------- Pin Definitions --------------------
#define IR1 D5             // First IR sensor input
#define IR2 D6             // Second IR sensor input
#define RED_LED D1         // Red LED to indicate rash driving
#define GREEN_LED D2       // Green LED to indicate safe driving

// Create a software serial port to communicate with SIM800L (RX, TX)
SoftwareSerial sim800(D7, D8);

// -------------------- Global Variables --------------------
unsigned long startTime = 0;        // To record time when IR1 is triggered
bool IR1_triggered = false;         // Tracks if IR1 has been triggered
bool waiting_for_IR2 = false;       // Wait state for IR2 to detect the next vehicle
bool smsSent = false;               // Prevents duplicate SMS for the same event

// -------------------- Setup Function --------------------
void setup() {
  // Set pin modes
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Turn off both LEDs at startup
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  // Initialize serial communication
  Serial.begin(9600);
  sim800.begin(9600);
  delay(2000);  // Allow SIM800L to initialize

  // Check if SIM800L is working
  checkSIMStatus();
}

// -------------------- Main Loop --------------------
void loop() {
  int ir1State = digitalRead(IR1);  // Read IR1 sensor
  int ir2State = digitalRead(IR2);  // Read IR2 sensor

  // IR1 is triggered - begin timing
  if (ir1State == LOW && !IR1_triggered && !waiting_for_IR2) {
    startTime = millis();             // Record the time
    IR1_triggered = true;            // Mark IR1 as triggered
    waiting_for_IR2 = true;          // Now wait for IR2
    smsSent = false;                 // Reset SMS flag for this cycle
    Serial.println("IR1 triggered - Timing started");
    delay(300);                      // Debounce delay
  }

  // IR2 is triggered after a delay of at least 0.5 seconds
  if (ir2State == LOW && waiting_for_IR2 && millis() - startTime > 500) {
    unsigned long duration = (millis() - startTime) / 1000;  // Calculate time taken in seconds
    Serial.print("IR2 triggered - Time taken: ");
    Serial.print(duration);
    Serial.println(" seconds");

    // If time between IR1 and IR2 is less than 5 seconds → rash driving
    if (duration < 5) {
      digitalWrite(RED_LED, HIGH);    // Turn on red LED
      digitalWrite(GREEN_LED, LOW);   // Ensure green LED is off
      Serial.println("Rash Driving Detected!");

      // Send SMS only once per detection
      if (!smsSent) {
        sendSMS("+919008383803", "Rash Driving Detected!");
        smsSent = true;
      }
    } else {
      // Otherwise, it's considered safe driving
      digitalWrite(RED_LED, LOW);     // Turn off red LED
      digitalWrite(GREEN_LED, HIGH);  // Turn on green LED
      Serial.println("Safe Driving");
    }

    delay(3000);  // Allow LED display to show result
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);

    // Reset flags for the next cycle
    IR1_triggered = false;
    waiting_for_IR2 = false;
    delay(500);   // Final debounce delay
  }
}

// -------------------- Function to Send SMS --------------------
void sendSMS(String number, String message) {
  Serial.println("Sending SMS...");
  sim800.println("AT+CMGF=1");     // Set SMS to text mode
  delay(1000);
  sim800.print("AT+CMGS=\"");
  sim800.print(number);            // Phone number
  sim800.println("\"");
  delay(2000);                     // Wait for > prompt
  sim800.print(message);          // Message body
  delay(500);
  sim800.write(26);               // Ctrl+Z character to send SMS
  delay(5000);                    // Wait for sending to complete
  Serial.println("SMS Sent to " + number);
}

// -------------------- Function to Check SIM800L Status --------------------
void checkSIMStatus() {
  Serial.println("Checking SIM800L...");
  sim800.println("AT");
  delay(1000);
  if (sim800.available()) {
    // Display SIM800L response
    while (sim800.available()) {
      Serial.write(sim800.read());
    }
    Serial.println();

    // Additional checks
    sim800.println("AT+CSQ");     // Check signal strength
    delay(1000);
    sim800.println("AT+CREG?");   // Check network registration
    delay(1000);
    sim800.println("AT+CMGF=1");  // Set text mode for SMS
    delay(1000);
  } else {
    Serial.println("SIM800L not responding. Check power and wiring.");
  }
}
