#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Initialize LCD at I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WIRING (same for both boards):
// E5 TX -> Pin 10
// E5 RX -> Pin 11
// GND -> GND, VCC -> 3.3V or 5V

// Both boards use SoftwareSerial on pins 10,11
SoftwareSerial loraSerial(10, 11);  // RX=10, TX=11
#define LORA_SERIAL loraSerial

// Simple buffer for responses - small for ATmega168, larger for Nano Every
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  char respBuf[32];
#else
  char respBuf[80];
#endif

// For LCD, write directly to avoid copying strings
void lcdPrint(const __FlashStringHelper* line1, const __FlashStringHelper* line2 = nullptr) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  if (line2) {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
}

void displayLCD(const char* line1, const char* line2 = "") {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  if (line2[0] != '\0') {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
}

// Send AT command and wait for response - returns true if got any response
bool sendAT(const char* cmd, unsigned int timeout = 1000) {
  // Clear RX buffer
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  
  LORA_SERIAL.println(cmd);
  
  unsigned long start = millis();
  byte idx = 0;
  
  while (millis() - start < timeout && idx < 31) {
    if (LORA_SERIAL.available()) {
      respBuf[idx++] = LORA_SERIAL.read();
    }
  }
  respBuf[idx] = '\0';
  
  return (idx > 0);
}

// ============ TRANSMITTER CODE (ATmega168) ============
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

int count = 0;
unsigned long lastTxTime = 0;
unsigned long stateStartTime = 0;
byte txState = 0;  // 0=idle, 1=sending, 2=reading response, 3=displaying result

void setup() {
  LORA_SERIAL.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  lcdPrint(F("TRANSMITTER"), F("Starting..."));
  delay(2000);  // Wait for LoRa-E5 to boot
  
  // Configure LoRa
  sendAT("AT+MODE=TEST", 1500);
  sendAT("AT+TEST=RFCFG,915,SF12,125,15,15,22,ON,OFF,OFF", 2000);
  
  lcdPrint(F("TX every 5s"), F("MAX RANGE"));
  lastTxTime = millis() - 5000;  // Trigger first TX immediately
}

void loop() {
  unsigned long now = millis();
  
  switch (txState) {
    case 0:  // Idle - wait for next TX time
      if (now - lastTxTime >= 5000) {
        lastTxTime = now;  // Reset timer at START of transmission
        count++;
        char line1[17];
        snprintf(line1, 17, "TX #%d", count);
        displayLCD(line1, "Sending...");
        
        // Send test packet
        LORA_SERIAL.print(F("AT+TEST=TXLRSTR,\"TEST"));
        LORA_SERIAL.print(count);
        LORA_SERIAL.println(F("\""));
        
        stateStartTime = now;
        txState = 1;
      }
      break;
      
    case 1:  // Wait for TX to complete
      if (now - stateStartTime >= 2000) {
        // Read any response
        byte idx = 0;
        while (LORA_SERIAL.available() && idx < 31) {
          respBuf[idx++] = LORA_SERIAL.read();
        }
        respBuf[idx] = '\0';
        
        char line1[17];
        snprintf(line1, 17, "TX #%d", count);
        if (strstr(respBuf, "DONE")) {
          displayLCD(line1, "TX OK!");
        } else {
          displayLCD(line1, "TX sent");
        }
        
        stateStartTime = now;
        txState = 2;
      }
      break;
      
    case 2:  // Display result briefly
      if (now - stateStartTime >= 1000) {
        txState = 0;
      }
      break;
  }
}

// ============ RECEIVER CODE (Nano Every) ============
#else

// Nano Every (Receiver)
// Same wiring as transmitter:
// E5 TX -> Pin 10
// E5 RX -> Pin 11

int rxCount = 0;
unsigned long lastRxTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastCharTime = 0;
byte respIdx = 0;
bool receiving = false;

void setup() {
  Serial.begin(9600);
  LORA_SERIAL.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  displayLCD("RECEIVER", "Starting...");
  delay(2000);  // Wait for LoRa-E5 to boot
  
  // Clear buffer and configure LoRa
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  sendAT("AT+MODE=TEST", 1500);
  sendAT("AT+TEST=RFCFG,915,SF12,125,15,15,22,ON,OFF,OFF", 2000);
  
  // Start RX mode
  LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
  
  displayLCD("Waiting for", "messages...");
  Serial.println(F("\n=== LISTENING FOR PACKETS ==="));
}

void loop() {
  unsigned long now = millis();
  
  // Non-blocking serial read - collect chars as they arrive
  while (LORA_SERIAL.available()) {
    char c = LORA_SERIAL.read();
    if (respIdx < 79) {
      respBuf[respIdx++] = c;
    }
    lastCharTime = now;
    receiving = true;
  }
  
  // If we were receiving and 100ms passed with no new chars, process the message
  if (receiving && (now - lastCharTime >= 100)) {
    respBuf[respIdx] = '\0';
    receiving = false;
    
    // Show everything received
    if (respIdx > 0) {
      Serial.print(F("RX: "));
      Serial.println(respBuf);
    }
    
    // Check for received LoRa packet
    if (strstr(respBuf, "RX \"") || strstr(respBuf, "RSSI")) {
      rxCount++;
      lastRxTime = now;
      char line1[17];
      snprintf(line1, 17, "RX #%d", rxCount);
      displayLCD(line1, "Time: 0.0s");
      Serial.print(F("*** PACKET "));
      Serial.print(rxCount);
      Serial.println(F(" ***"));
      
      // Re-enter RX mode
      LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
    }
    
    respIdx = 0;  // Reset for next message
  }
  
  // Update time display every 100ms (non-blocking)
  if (lastRxTime > 0 && !receiving && (now - lastDisplayUpdate >= 100)) {
    lastDisplayUpdate = now;
    unsigned long elapsed = (now - lastRxTime) / 100;
    char line1[17], line2[17];
    snprintf(line1, 17, "RX #%d", rxCount);
    snprintf(line2, 17, "Time: %lu.%lus", elapsed / 10, elapsed % 10);
    displayLCD(line1, line2);
  }
}

#endif