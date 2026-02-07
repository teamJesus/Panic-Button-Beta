#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

// Initialize OLED at I2C address 0x3C
#define OLED_ADDR 0x3C
SSD1306AsciiWire oled;

// Buffer size and serial config depends on chip
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  // ATmega168/328: Use SoftwareSerial on pins 10,11
  // WIRING: E5 TX -> Pin 10, E5 RX -> Pin 11
  #include <SoftwareSerial.h>
  SoftwareSerial loraSerial(10, 11);  // RX=10, TX=11
  #define LORA_SERIAL loraSerial
  char respBuf[96];
  #define DEVICE_NAME "DEV-A"
  const unsigned long DISPLAY_INTERVAL_MS = 500;
  const bool DEBUG_SERIAL = true;
  const unsigned long RX_SILENCE_MS = 200;
#else
  // Nano Every (ATmega4809): Use hardware Serial1 on pins 0,1
  // WIRING: E5 TX -> Pin 0 (RX), E5 RX -> Pin 1 (TX)
  #define LORA_SERIAL Serial1
  char respBuf[64];
  #define DEVICE_NAME "DEV-B"
  const unsigned long DISPLAY_INTERVAL_MS = 100;
  const bool DEBUG_SERIAL = false;
  const unsigned long RX_SILENCE_MS = 100;
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
char lastSender[17] = "Waiting";
unsigned long buttonMsgUntil = 0;

const byte BUTTON_PIN = 5;  // D5
const unsigned long DEBOUNCE_MS = 5;
const unsigned long MIN_PRESS_MS = 5;
bool buttonState = HIGH;  // INPUT_PULLUP idle
bool lastButtonReading = HIGH;
unsigned long lastButtonChangeAt = 0;
unsigned long lowStartAt = 0;
bool seenLow = false;
bool txRequested = false;
volatile bool buttonIsrFired = false;
unsigned long lastIsrAt = 0;

// Display text on OLED row (row 0-3 for 128x64 with 2X font)
void displayLine(byte row, const char* text) {
  oled.setCursor(0, row * 2);  // 2X font uses 2 rows per line
  oled.print(text);
  oled.clearToEOL();  // Clear rest of line
}

void setLastSender(const char* name) {
  strncpy(lastSender, name, sizeof(lastSender) - 1);
  lastSender[sizeof(lastSender) - 1] = '\0';
}

bool isHexChar(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

bool decodeHexPayload(const char* payload, char* out, size_t outSize) {
  size_t len = strlen(payload);
  if (len < 2 || (len % 2) != 0) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    if (!isHexChar(payload[i])) {
      return false;
    }
  }
  size_t outLen = len / 2;
  if (outLen >= outSize) {
    outLen = outSize - 1;
  }
  for (size_t i = 0; i < outLen; i++) {
    int hi = hexValue(payload[i * 2]);
    int lo = hexValue(payload[i * 2 + 1]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    out[i] = (char)((hi << 4) | lo);
  }
  out[outLen] = '\0';
  return true;
}

void updateSenderFromPayload(const char* payload) {
  char decoded[33];
  const char* parsed = payload;
  if (decodeHexPayload(payload, decoded, sizeof(decoded))) {
    parsed = decoded;
  }

  const char* end = strrchr(parsed, '-');
  size_t len = end ? (size_t)(end - parsed) : strlen(parsed);
  if (len == 0) {
    return;
  }
  if (len > sizeof(lastSender) - 1) {
    len = sizeof(lastSender) - 1;
  }
  memcpy(lastSender, parsed, len);
  lastSender[len] = '\0';
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#if !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega328P__)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), []() { buttonIsrFired = true; }, FALLING);
#endif
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  if (DEBUG_SERIAL) {
    Serial.begin(9600);
    Serial.println(F("DEV-A debug enabled"));
  }
#endif
  Wire.begin();
  oled.begin(&Adafruit128x64, OLED_ADDR);
  oled.setFont(Adafruit5x7);
  oled.set2X();  // Double size for readability
  oled.clear();
  
  displayLine(0, DEVICE_NAME);
  displayLine(1, "Init LoRa...");
  
  LORA_SERIAL.begin(9600);
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  loraSerial.listen();
#endif
  delay(2000);  // Wait for LoRa-E5 to boot
  
  // Clear buffer and configure LoRa
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  
  // Set TEST mode
  LORA_SERIAL.println(F("AT+MODE=TEST"));
  delay(1000);
  
  // Read response to check if LoRa is responding
  byte cnt = 0;
  while (LORA_SERIAL.available() && cnt < sizeof(respBuf)-1) {
    respBuf[cnt++] = LORA_SERIAL.read();
  }
  respBuf[cnt] = '\0';
  
  // Show response on display (truncated)
  if (cnt > 0) {
    displayLine(1, "LoRa OK");
  } else {
    displayLine(1, "No response!");
  }
  delay(1000);
  
  // Configure RF - max range
  LORA_SERIAL.println(F("AT+TEST=RFCFG,915,SF12,125,15,15,22,ON,OFF,OFF"));
  delay(500);
  while (LORA_SERIAL.available()) LORA_SERIAL.read();
  
  // Start in RX mode
  LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
  delay(100);
  lastTxTime = millis();
  lastRxTime = lastTxTime;
  state = 0;
  
  displayLine(1, "Ready");
  displayLine(2, "TX from:");
  setLastSender("Waiting");
  displayLine(3, lastSender);
}

void loop() {
  unsigned long now = millis();

#if !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega328P__)
  if (buttonIsrFired) {
    buttonIsrFired = false;
    if (now - lastIsrAt >= 20) {
      lastIsrAt = now;
      txRequested = true;
      buttonMsgUntil = now + 1000;
    }
  }
#else
  // Debounced button read (active low)
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) {
    lastButtonChangeAt = now;
    lastButtonReading = reading;
  }
  if (now - lastButtonChangeAt >= DEBOUNCE_MS) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        seenLow = true;
        lowStartAt = now;
      } else {
        if (seenLow && (now - lowStartAt >= MIN_PRESS_MS)) {
          txRequested = true;  // Single send per press
          buttonMsgUntil = now + 1000;
        }
        seenLow = false;
      }
    }
  }
#endif
  
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
  if (receiving && (now - lastCharTime >= RX_SILENCE_MS)) {
    respBuf[respIdx] = '\0';
    receiving = false;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    if (DEBUG_SERIAL) {
      Serial.print(F("RX RAW: "));
      Serial.println(respBuf);
    }
#endif
    
    // Check for received LoRa packet
    if (strstr(respBuf, "RX \"") || strstr(respBuf, "RSSI")) {
      rxCount++;
      lastRxTime = now;
      
      // Parse RSSI value (format: RSSI:-XX)
      char* rssiPtr = strstr(respBuf, "RSSI:");
      if (rssiPtr) {
        lastRssi = atoi(rssiPtr + 5);  // Skip "RSSI:"
      }

      char* quoteStart = strchr(respBuf, '"');
      if (quoteStart) {
        char* quoteEnd = strchr(quoteStart + 1, '"');
        if (quoteEnd && quoteEnd > quoteStart + 1) {
          char payload[33];
          size_t len = (size_t)(quoteEnd - (quoteStart + 1));
          if (len > sizeof(payload) - 1) {
            len = sizeof(payload) - 1;
          }
          memcpy(payload, quoteStart + 1, len);
          payload[len] = '\0';
          updateSenderFromPayload(payload);
        }
      }
      
      // Re-enter RX mode if we're in RX state
      if (state == 0) {
        LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
        loraSerial.listen();
#endif
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
    case 0:  // RX mode - wait for button press
      if (txRequested && (now - lastTxTime >= 200)) {
        lastTxTime = now;
        txRequested = false;
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
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
        loraSerial.listen();
#endif
        state = 0;
      }
      break;
  }
  
  // Update display frequently, but avoid colliding with serial reads
  if (!receiving && (now - lastDisplayUpdate >= DISPLAY_INTERVAL_MS)) {
    lastDisplayUpdate = now;

    if (now - lastRxTime >= 2000) {
      setLastSender("Waiting");
    }

    displayLine(0, DEVICE_NAME);
    displayLine(1, (now < buttonMsgUntil) ? "Sending" : "Ready");
    displayLine(2, "TX from:");
    displayLine(3, lastSender);
  }
}
