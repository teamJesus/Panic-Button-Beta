#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

#define OLED_ADDR 0x3C
SSD1306AsciiWire oled;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #include <SoftwareSerial.h>
  SoftwareSerial loraSerial(10, 11);  // RX=10, TX=11
  #define LORA_SERIAL loraSerial
  char respBuf[48];
  const unsigned long DISPLAY_INTERVAL_MS = 150;
  const unsigned long RX_SILENCE_MS = 200;
  const bool DEBUG_SERIAL = false;
  const bool ENABLE_LORA = true;
#else
  #define LORA_SERIAL Serial1
  char respBuf[128];
  const unsigned long DISPLAY_INTERVAL_MS = 75;
  const unsigned long RX_SILENCE_MS = 100;
  const bool DEBUG_SERIAL = false;
  const bool ENABLE_LORA = true;
#endif


unsigned long lastRxTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastCharTime = 0;
unsigned long txStartAt = 0;
bool txInProgress = false;
const unsigned long TX_DONE_TIMEOUT_MS = 1500;
int rxCount = 0;
int lastRssi = -120;
byte respIdx = 0;
bool receiving = false;
char lastSender[17] = "Waiting";
char rxDisplayName[17] = "";
unsigned long rxDisplayUntil = 0;

const byte BUTTON_PINS[] = {2, 3, 4, 5};
const byte BUTTON_COUNT = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);
const byte BTN1_IDX = 0;  // D2
const byte BTN2_IDX = 1;  // D3
const byte BTN3_IDX = 2;  // D4
const byte BTN4_IDX = 3;  // D5
const unsigned long DEBOUNCE_MS = 5;
const unsigned long MIN_PRESS_MS = 5;
const unsigned long LONG_PRESS_MS = 2000;
bool buttonState[BUTTON_COUNT] = {HIGH, HIGH, HIGH, HIGH};
bool lastButtonReading[BUTTON_COUNT] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastButtonChangeAt[BUTTON_COUNT] = {0, 0, 0, 0};
unsigned long lowStartAt[BUTTON_COUNT] = {0, 0, 0, 0};
bool seenLow[BUTTON_COUNT] = {false, false, false, false};
bool buttonShort[BUTTON_COUNT] = {false, false, false, false};
bool buttonLong[BUTTON_COUNT] = {false, false, false, false};
bool longPressFired[BUTTON_COUNT] = {false, false, false, false};

const byte NAME_MAX_LEN = 16;
const int NAME_EEPROM_ADDR = 0;
char userName[NAME_MAX_LEN + 1] = "add name";
char editName[NAME_MAX_LEN + 1] = "add name";
byte cursorIndex = 0;
const char kNameChars[] = "abcdefghijklmnopqrstuvwxyz ";
const unsigned long CURSOR_BLINK_MS = 300;

char prevLine0[17] = "";
char prevLine1[17] = "";
char prevLine2[17] = "";
char prevLine3[17] = "";

enum ScreenMode {
  MODE_MAIN = 0,
  MODE_NAMING = 1
};
ScreenMode screenMode = MODE_MAIN;

void displayLine(byte row, const char* text) {
  oled.setCursor(0, row * 2);
  oled.print(text);
  oled.clearToEOL();
}

void updateLineIfChanged(byte row, const char* text, char* prev) {
  if (strncmp(prev, text, 16) != 0) {
    displayLine(row, text);
    strncpy(prev, text, 16);
    prev[16] = '\0';
  }
}

void setLastSender(const char* name) {
  strncpy(lastSender, name, sizeof(lastSender) - 1);
  lastSender[sizeof(lastSender) - 1] = '\0';
}

void setRxDisplayName(const char* name, unsigned long now) {
  strncpy(rxDisplayName, name, sizeof(rxDisplayName) - 1);
  rxDisplayName[sizeof(rxDisplayName) - 1] = '\0';
  rxDisplayUntil = now + 1000;
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

void sendNamePacket(const char* name) {
  char payload[NAME_MAX_LEN + 1];
  strncpy(payload, name, NAME_MAX_LEN);
  payload[NAME_MAX_LEN] = '\0';

  char cmd[40 + NAME_MAX_LEN];
  snprintf(cmd, sizeof(cmd), "AT+TEST=TXLRSTR,\"%s\"", payload);
  LORA_SERIAL.println(cmd);
  if (DEBUG_SERIAL) {
    Serial.print(F("TX CMD: "));
    Serial.println(cmd);
  }
}

void loadUserName() {
  byte first = EEPROM.read(NAME_EEPROM_ADDR);
  if (first == 0xFF || first == 0x00) {
    strncpy(userName, "add name", sizeof(userName) - 1);
    userName[sizeof(userName) - 1] = '\0';
    return;
  }

  for (byte i = 0; i < NAME_MAX_LEN; i++) {
    char c = (char)EEPROM.read(NAME_EEPROM_ADDR + i);
    if (c == '\0' || c == 0xFF) {
      userName[i] = '\0';
      break;
    }
    userName[i] = c;
    if (i == NAME_MAX_LEN - 1) {
      userName[NAME_MAX_LEN] = '\0';
    }
  }

  if (userName[0] == '\0') {
    strncpy(userName, "add name", sizeof(userName) - 1);
    userName[sizeof(userName) - 1] = '\0';
  }

  bool hasValid = false;
  for (byte i = 0; i < NAME_MAX_LEN; i++) {
    char c = userName[i];
    if (c == '\0') {
      break;
    }
    bool ok = (c >= 'a' && c <= 'z') || c == ' ';
    if (!ok) {
      userName[i] = ' ';
    } else if (c != ' ') {
      hasValid = true;
    }
  }

  if (!hasValid) {
    strncpy(userName, "add name", sizeof(userName) - 1);
    userName[sizeof(userName) - 1] = '\0';
  }
}

void saveUserName(const char* name) {
  char trimmed[NAME_MAX_LEN + 1];
  strncpy(trimmed, name, NAME_MAX_LEN);
  trimmed[NAME_MAX_LEN] = '\0';

  int end = NAME_MAX_LEN - 1;
  while (end >= 0 && trimmed[end] == ' ') {
    trimmed[end] = '\0';
    end--;
  }

  if (trimmed[0] == '\0') {
    strncpy(trimmed, "add name", sizeof(trimmed) - 1);
    trimmed[sizeof(trimmed) - 1] = '\0';
  }

  for (byte i = 0; i < NAME_MAX_LEN; i++) {
    char c = trimmed[i];
    if (c == '\0') {
      EEPROM.update(NAME_EEPROM_ADDR + i, 0);
      break;
    }
    EEPROM.update(NAME_EEPROM_ADDR + i, c);
  }

  strncpy(userName, trimmed, sizeof(userName) - 1);
  userName[sizeof(userName) - 1] = '\0';
}

char nextNameChar(char current, int direction) {
  const int count = (int)(sizeof(kNameChars) - 1);
  int index = 0;
  for (int i = 0; i < count; i++) {
    if (kNameChars[i] == current) {
      index = i;
      break;
    }
  }
  index = (index + direction + count) % count;
  return kNameChars[index];
}

void enterNamingMode() {
  screenMode = MODE_NAMING;
  oled.clear();
  strncpy(editName, userName, NAME_MAX_LEN);
  editName[NAME_MAX_LEN] = '\0';
  size_t len = strlen(editName);
  for (size_t i = len; i < NAME_MAX_LEN; i++) {
    editName[i] = ' ';
  }
  editName[NAME_MAX_LEN] = '\0';
  cursorIndex = 0;
}

void leaveNamingMode(bool save) {
  if (save) {
    saveUserName(editName);
  }
  screenMode = MODE_MAIN;
  oled.clear();
}

void setup() {
  for (byte i = 0; i < BUTTON_COUNT; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  Serial.begin(9600);
  Serial.println(F("Button test: D2=btn1 D3=btn2 D4=btn3 D5=btn4"));

  Wire.begin();
  oled.begin(&Adafruit128x64, OLED_ADDR);
  oled.setFont(Adafruit5x7);
  oled.set2X();
  oled.clear();

  loadUserName();
  displayLine(0, userName);
  displayLine(1, "");
  displayLine(2, "");
  displayLine(3, "");
  strncpy(prevLine0, userName, 16);
  prevLine0[16] = '\0';
  prevLine1[0] = '\0';
  prevLine2[0] = '\0';
  prevLine3[0] = '\0';

  if (ENABLE_LORA) {
    LORA_SERIAL.begin(9600);
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    loraSerial.listen();
#endif
    delay(2000);

    while (LORA_SERIAL.available()) LORA_SERIAL.read();

    LORA_SERIAL.println(F("AT+MODE=TEST"));
    delay(1000);

    LORA_SERIAL.println(F("AT+TEST=RFCFG,915,SF12,125,15,15,22,ON,OFF,OFF"));
    delay(500);
    while (LORA_SERIAL.available()) LORA_SERIAL.read();

    LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
    delay(100);
  }
  lastRxTime = millis();

  setLastSender("Waiting");
}

void loop() {
  unsigned long now = millis();

  for (byte i = 0; i < BUTTON_COUNT; i++) {
    bool reading = digitalRead(BUTTON_PINS[i]);
    if (reading != lastButtonReading[i]) {
      lastButtonChangeAt[i] = now;
      lastButtonReading[i] = reading;
    }
    if (now - lastButtonChangeAt[i] >= DEBOUNCE_MS) {
      if (reading != buttonState[i]) {
        buttonState[i] = reading;
        if (buttonState[i] == LOW) {
          seenLow[i] = true;
          lowStartAt[i] = now;
          longPressFired[i] = false;
        } else {
          if (seenLow[i] && (now - lowStartAt[i] >= MIN_PRESS_MS)) {
            if (!longPressFired[i]) {
              buttonShort[i] = true;
            }
          }
          seenLow[i] = false;
        }
      }
    }

    if (buttonState[i] == LOW && seenLow[i] && !longPressFired[i]) {
      if (now - lowStartAt[i] >= LONG_PRESS_MS) {
        longPressFired[i] = true;
        buttonLong[i] = true;
      }
    }
  }

  for (byte i = 0; i < BUTTON_COUNT; i++) {
    if (buttonShort[i]) {
      Serial.print(F("SHORT Btn"));
      Serial.println(i + 1);
    }
    if (buttonLong[i]) {
      Serial.print(F("LONG Btn"));
      Serial.println(i + 1);
    }
  }

  if (ENABLE_LORA) {
    while (LORA_SERIAL.available()) {
      char c = LORA_SERIAL.read();
      if (respIdx < sizeof(respBuf) - 1) {
        respBuf[respIdx++] = c;
      }
      lastCharTime = now;
      receiving = true;
    }

    if (receiving && (now - lastCharTime >= RX_SILENCE_MS)) {
      respBuf[respIdx] = '\0';
      receiving = false;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    if (DEBUG_SERIAL) {
      Serial.print(F("RX RAW: "));
      Serial.println(respBuf);
    }
#endif

    if (DEBUG_SERIAL && respBuf[0] != '\0') {
      Serial.print(F("RX: "));
      Serial.println(respBuf);
    }

      if (strstr(respBuf, "RX \"") || strstr(respBuf, "RSSI")) {
        rxCount++;
        lastRxTime = now;

        char* rssiPtr = strstr(respBuf, "RSSI:");
        if (rssiPtr) {
          lastRssi = atoi(rssiPtr + 5);
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
            setRxDisplayName(lastSender, now);
          }
        }

        LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
        loraSerial.listen();
#endif
      }

          if (strstr(respBuf, "TX DONE")) {
        txInProgress = false;
        LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
      #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
        loraSerial.listen();
      #endif
          }

      respIdx = 0;
    }
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = now;

    if (now - lastRxTime >= 2000) {
      setLastSender("Waiting");
    }

    if (screenMode == MODE_MAIN) {
      char line2[17] = "";
      if (rxDisplayUntil != 0 && now < rxDisplayUntil) {
        snprintf(line2, sizeof(line2), "From:%s", rxDisplayName);
      }
      updateLineIfChanged(0, userName, prevLine0);
      updateLineIfChanged(1, "", prevLine1);
      updateLineIfChanged(2, line2, prevLine2);
      updateLineIfChanged(3, "", prevLine3);
    } else {
      updateLineIfChanged(0, editName, prevLine0);
      char arrowLine[NAME_MAX_LEN + 1];
      bool blinkOn = ((now / CURSOR_BLINK_MS) % 2) == 0;
      for (byte i = 0; i < NAME_MAX_LEN; i++) {
        arrowLine[i] = (i == cursorIndex && blinkOn) ? '^' : ' ';
      }
      arrowLine[NAME_MAX_LEN] = '\0';
      updateLineIfChanged(1, arrowLine, prevLine1);
      updateLineIfChanged(2, "", prevLine2);
      updateLineIfChanged(3, "", prevLine3);
    }
  }

  if (screenMode == MODE_MAIN) {
    if (buttonShort[BTN1_IDX] && ENABLE_LORA) {
      buttonShort[BTN1_IDX] = false;
      if (!txInProgress) {
        sendNamePacket(userName);
        txStartAt = now;
        txInProgress = true;
      }
    }
    if (buttonLong[BTN4_IDX]) {
      buttonLong[BTN4_IDX] = false;
      enterNamingMode();
    }
  } else {
    if (buttonLong[BTN4_IDX]) {
      buttonLong[BTN4_IDX] = false;
      leaveNamingMode(true);
    }

    if (buttonLong[BTN3_IDX]) {
      buttonLong[BTN3_IDX] = false;
      for (byte i = 0; i < NAME_MAX_LEN; i++) {
        editName[i] = ' ';
      }
      editName[NAME_MAX_LEN] = '\0';
      cursorIndex = 0;
    }

    if (buttonShort[BTN2_IDX]) {
      buttonShort[BTN2_IDX] = false;
      editName[cursorIndex] = nextNameChar(editName[cursorIndex], 1);
    }
    if (buttonShort[BTN1_IDX]) {
      buttonShort[BTN1_IDX] = false;
      editName[cursorIndex] = nextNameChar(editName[cursorIndex], -1);
    }
    if (buttonShort[BTN4_IDX]) {
      buttonShort[BTN4_IDX] = false;
      cursorIndex = (cursorIndex + 1) % NAME_MAX_LEN;
    }
    if (buttonShort[BTN3_IDX]) {
      buttonShort[BTN3_IDX] = false;
      cursorIndex = (cursorIndex + NAME_MAX_LEN - 1) % NAME_MAX_LEN;
    }
  }

  for (byte i = 0; i < BUTTON_COUNT; i++) {
    buttonShort[i] = false;
    buttonLong[i] = false;
  }

  if (txInProgress && (now - txStartAt >= TX_DONE_TIMEOUT_MS)) {
    LORA_SERIAL.println(F("AT+TEST=RXLRPKT"));
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    loraSerial.listen();
#endif
    txInProgress = false;
  }

  // No OLED button test output.
}
