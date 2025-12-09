#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Initialize LCD at I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WIRING:
// ATmega168 (Transmitter): E5 TX -> Pin 10, E5 RX -> Pin 11
// Nano Every (Receiver):   E5 TX -> Pin 0,  E5 RX -> Pin 1
// Both: GND -> GND, VCC -> 3.3V

// For ATmega168: Use SoftwareSerial on pins 10,11
// For Nano Every: Use Serial1 (hardware UART on pins 0,1)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  SoftwareSerial loraSerial(10, 11);  // RX=10, TX=11
  #define LORA_SERIAL loraSerial
#else
  #define LORA_SERIAL Serial1
#endif

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

void setup() {
  LORA_SERIAL.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  lcdPrint(F("ATmega168"), F("TRANSMITTER"));
  delay(2000);
  
  lcdPrint(F("Init LoRa..."), F("Please wait"));
  delay(2000);
  
  lcdPrint(F("Sending AT..."));
  
  if (sendAT("AT", 1500)) {
    lcdPrint(F("Got response!"));
    delay(1500);
  } else {
    lcdPrint(F("No response"), F("Check wiring"));
    delay(2000);
    lcdPrint(F("E5 TX->Pin10"), F("E5 RX->Pin11"));
    while(1) { delay(1000); }
  }
  
  lcdPrint(F("Set TEST mode"));
  sendAT("AT+MODE=TEST", 1500);
  delay(1000);
  
  lcdPrint(F("Config RF..."));
  // Set frequency to 915MHz
  sendAT("AT+TEST=RFCFG,915,SF7,125,12,15,14,ON,OFF,OFF", 2000);
  delay(1000);
  
  lcdPrint(F("Ready!"), F("TX every 5s"));
  delay(1000);
}

void loop() {
  count++;
  
  char line1[17];
  snprintf(line1, 17, "TX #%d", count);
  displayLCD(line1, "Sending...");
  
  // Send test packet
  LORA_SERIAL.print(F("AT+TEST=TXLRSTR,\"TEST"));
  LORA_SERIAL.print(count);
  LORA_SERIAL.println(F("\""));
  
  delay(2000);
  
  // Read response
  byte idx = 0;
  unsigned long start = millis();
  while (millis() - start < 2000 && idx < 31) {
    if (LORA_SERIAL.available()) {
      respBuf[idx++] = LORA_SERIAL.read();
    }
  }
  respBuf[idx] = '\0';
  
  if (strstr(respBuf, "DONE")) {
    displayLCD(line1, "TX OK!");
  } else {
    displayLCD(line1, "TX sent");
  }
  
  delay(3000);
}

// ============ RECEIVER CODE (Nano Every) ============
#else

// Nano Every uses Serial1 (hardware UART on pins 0,1)
// Same wiring as transmitter:
// E5 TX -> Pin 0
// E5 RX -> Pin 1

int rxCount = 0;

void setup() {
  Serial.begin(9600);
  LORA_SERIAL.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  displayLCD("Nano Every", "RECEIVER");
  Serial.println("=== NANO EVERY RECEIVER ===");
  Serial.println("Wiring: E5 TX->Pin0, E5 RX->Pin1");
  delay(2000);
  
  displayLCD("Init LoRa...");
  Serial.println("Waiting for LoRa-E5 to boot...");
  delay(3000);
  
  // Clear buffer
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  
  // Test AT
  displayLCD("Testing AT...");
  if (sendAT("AT", 2000)) {
    Serial.print("AT Response: ");
    Serial.println(respBuf);
    displayLCD("AT OK!");
    delay(1000);
  } else {
    displayLCD("AT Failed!", "Check wiring");
    Serial.println("No response - check wiring!");
    delay(3000);
  }
  
  // Set test mode
  displayLCD("Set TEST mode");
  Serial.println("Setting TEST mode...");
  sendAT("AT+MODE=TEST", 2000);
  Serial.print("MODE response: ");
  Serial.println(respBuf);
  delay(1000);
  
  // Configure RF - MUST MATCH TRANSMITTER!
  displayLCD("Config RF...");
  Serial.println("Configuring RF (915MHz, SF7, 125kHz)...");
  // Set frequency to 915MHz
  sendAT("AT+TEST=RFCFG,915,SF7,125,12,15,14,ON,OFF,OFF", 2000);
  Serial.print("RFCFG response: ");
  Serial.println(respBuf);
  delay(1000);
  
  // Start continuous RX mode
  displayLCD("Start RX...");
  Serial.println("Starting RX mode...");
  LORA_SERIAL.println("AT+TEST=RXLRPKT");
  delay(500);
  
  displayLCD("Waiting for", "messages...");
  Serial.println("\n=== LISTENING FOR PACKETS ===");
  Serial.println("Make sure transmitter is running!");
}

void loop() {
  if (LORA_SERIAL.available()) {
    byte idx = 0;
    unsigned long start = millis();
    
    while (millis() - start < 500 && idx < 80) {
      if (LORA_SERIAL.available()) {
        char c = LORA_SERIAL.read();
        if (idx < 79) respBuf[idx++] = c;
        start = millis();
      }
    }
    respBuf[idx] = '\0';
    
    // Show everything received
    if (idx > 0) {
      Serial.print("RX: ");
      Serial.println(respBuf);
    }
    
    // Check for received LoRa packet
    if (strstr(respBuf, "RX \"") || strstr(respBuf, "RSSI")) {
      rxCount++;
      char line1[17];
      snprintf(line1, 17, "RX #%d", rxCount);
      displayLCD(line1, "Got packet!");
      Serial.print("*** PACKET ");
      Serial.print(rxCount);
      Serial.println(" ***");
      
      // Re-enter RX mode after receiving
      LORA_SERIAL.println("AT+TEST=RXLRPKT");
    }
  }
}

#endif