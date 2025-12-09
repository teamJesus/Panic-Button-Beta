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

// Buffer size depends on chip RAM
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  char respBuf[32];
  #define DEVICE_NAME "DEV-A"
#else
  char respBuf[64];
  #define DEVICE_NAME "DEV-B"
#endif

// State machine
byte state = 0;  // 0=RX mode, 1=TX sending, 2=TX done, 3=back to RX
unsigned long stateStartTime = 0;
unsigned long lastTxTime = 0;
unsigned long lastRxTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastCharTime = 0;
int txCount = 0;
int rxCount = 0;
int lastRssi = -120;  // Last received signal strength
byte respIdx = 0;
bool receiving = false;

// Pad string to 16 chars to overwrite old content
void displayLine(byte row, const char* text) {
  lcd.setCursor(0, row);
  lcd.print(text);
  // Pad with spaces to clear rest of line
  for (byte i = strlen(text); i < 16; i++) {
    lcd.print(' ');
  }
}

void setup() {
  LORA_SERIAL.begin(9600);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  displayLine(0, DEVICE_NAME);
  displayLine(1, "Starting...");
  delay(2000);  // Wait for LoRa-E5 to boot
  
  // Clear buffer and configure LoRa
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  
  // Set TEST mode
  LORA_SERIAL.println(F("AT+MODE=TEST"));
  delay(500);
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  
  // Configure RF - max range
  LORA_SERIAL.println(F("AT+TEST=RFCFG,915,SF12,125,15,15,22,ON,OFF,OFF"));
  delay(500);
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  
  // Start in RX mode
  LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
  delay(100);
  lastTxTime = millis();
  state = 0;
}

void loop() {
  unsigned long now = millis();
  
  // Collect serial data (non-blocking)
  while (LORA_SERIAL.available()) {
    char c = LORA_SERIAL.read();
    if (respIdx < sizeof(respBuf) - 1) {
      respBuf[respIdx++] = c;
    }
    lastCharTime = now;
    receiving = true;
  }
  
  // Process received data after 100ms of silence
  if (receiving && (now - lastCharTime >= 100)) {
    respBuf[respIdx] = '\0';
    receiving = false;
    
    // Check for received LoRa packet
    if (strstr(respBuf, "RX \"") || strstr(respBuf, "RSSI")) {
      rxCount++;
      lastRxTime = now;
      
      // Parse RSSI value (format: RSSI:-XX)
      char* rssiPtr = strstr(respBuf, "RSSI:");
      if (rssiPtr) {
        lastRssi = atoi(rssiPtr + 5);  // Skip "RSSI:"
      }
      
      // Re-enter RX mode if we're in RX state
      if (state == 0) {
        LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
      }
    }
    
    // Check for TX done
    if (state == 1 && strstr(respBuf, "TX DONE")) {
      state = 2;
      stateStartTime = now;
    }
    
    respIdx = 0;
  }
  
  // State machine
  switch (state) {
    case 0:  // RX mode - wait for TX interval
      if (now - lastTxTime >= 5000) {
        lastTxTime = now;  // Reset at START so 5s is between TX starts
        txCount++;
        char cmd[40];
        snprintf(cmd, 40, "AT+TEST=TXLRSTR,\"%s-%d\"", DEVICE_NAME, txCount);
        LORA_SERIAL.println(cmd);
        
        stateStartTime = now;
        state = 1;
      }
      break;
      
    case 1:  // Waiting for TX to complete
      if (now - stateStartTime >= 3000) {
        // Timeout - move on anyway
        state = 2;
        stateStartTime = now;
      }
      break;
      
    case 2:  // TX done, brief pause
      if (now - stateStartTime >= 200) {
        // Back to RX mode
        LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
        state = 0;
      }
      break;
  }
  
  // Update display every 100ms (non-blocking)
  if (now - lastDisplayUpdate >= 100) {
    lastDisplayUpdate = now;
    
    char line[17];
    unsigned long elapsed = (now - lastRxTime) / 100;
    
    // Convert RSSI to percentage: -120dBm=0%, -30dBm=100%
    int sigPct = (lastRssi + 120) * 100 / 90;
    if (sigPct < 0) sigPct = 0;
    if (sigPct > 100) sigPct = 100;
    
    snprintf(line, 17, "Signal: %d%%", sigPct);
    displayLine(0, line);
    
    snprintf(line, 17, "Last: %lu.%lus", elapsed / 10, elapsed % 10);
    displayLine(1, line);
  }
}