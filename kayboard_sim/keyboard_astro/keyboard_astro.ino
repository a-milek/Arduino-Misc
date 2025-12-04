#include <Arduino.h>
#define ROWS 7
#define COLS 9
#define SCAN_INTERVAL 50
#define SCAN_DURATION 500

char matrix[ROWS][COLS] = {
  // 0   1    2   3   4  
  { '+','-',' ',' ',' ',' ',' ',' ',' ' },  // 0
  { ' ',' ','G','H','I',' ',' ',' ',' ' },  // 1
  { ' ',' ','A','B','C',' ',' ',' ',' ' },  // 2
  { ' ',' ','F','E','D',' ',' ',' ',' ' },   // 3
  { ' ',' ',' ',' ',' ','7','8','9',' ' },
  { ' ',' ',' ',' ',' ','1','2','3',' ' },
  { ' ',' ',' ',' ',' ','6','5','4',' ' },
};

struct PinStatus {
  uint8_t pin;
  bool changed;
  int lastState;
  int change_cnt;
};

struct Position {
  int8_t row;
  int8_t col;
};

#define OUT_PINS_CNT 8
static uint8_t out_pins[OUT_PINS_CNT] = {2,3,4,5,6,14,15,16};


#define SCANNING_PINS_CNT 7
static uint8_t scanning_pins[SCANNING_PINS_CNT] = {7,8,9,10,17,18,19};   


PinStatus pins[OUT_PINS_CNT + SCANNING_PINS_CNT];
Position c_pos={-1,-1};


boolean keyboard_connection() {
  for (uint8_t i = 0; i < OUT_PINS_CNT + SCANNING_PINS_CNT; i++) {
    pins[i].changed = false;
    pins[i].change_cnt = 0;
  }
  uint8_t idx = 0;

  for (uint8_t i = 0; i < OUT_PINS_CNT; i++, idx++) {
    pinMode(out_pins[i], INPUT_PULLUP);
    pins[idx] = { out_pins[i], false, digitalRead(out_pins[i]) };
  }

  for (uint8_t i = 0; i < SCANNING_PINS_CNT; i++, idx++) {
    pinMode(scanning_pins[i], INPUT_PULLUP);
    pins[idx] = { scanning_pins[i], false, digitalRead(scanning_pins[i]) };
  }

  unsigned long start = millis();

  while (millis() - start < SCAN_DURATION) {
    for (uint8_t i = 0; i < OUT_PINS_CNT + SCANNING_PINS_CNT; i++) {
      int current = digitalRead(pins[i].pin);
      if (current != pins[i].lastState) {
        pins[i].changed = true;
        pins[i].lastState = current;
        pins[i].change_cnt++;
      }
    }
    delay(SCAN_INTERVAL);
  }

  Serial.println("=== Keyboard test ===");
  for (uint8_t i = 0; i < OUT_PINS_CNT + SCANNING_PINS_CNT; i++) {
    Serial.print("Pin ");
    Serial.print(pins[i].pin);
    Serial.print(": ");
    Serial.println(pins[i].change_cnt);
  }

  bool allOutUnchanged = true;
  bool allScanChanged = true;

  for (uint8_t i = 0; i < OUT_PINS_CNT; i++) {
    if (pins[i].changed) {
      allOutUnchanged = false;
      break;
    }
  }

  for (uint8_t i = OUT_PINS_CNT; i < OUT_PINS_CNT + SCANNING_PINS_CNT; i++) {
    if (!pins[i].changed) {
      allScanChanged = false;
      break;
    }
  }

  if (allOutUnchanged && allScanChanged) {
    Serial.println("Everything is okay");
    return true;
  } else {
    Serial.println("ERROR");
    return false;
  }
}

Position find_in_matrix(char value) {
  for (int8_t row = 0; row < ROWS; row++) {
    for (int8_t col = 0; col < COLS; col++) {
      if (matrix[row][col] == value) {
        return { row, col };
      }
    }
  }
  return { -1, -1 };
}

void setup() {
  Serial.begin(9600);
  while (!keyboard_connection()) {
    Serial.println("Retrying keyboard connection...");
    delay(2000);
  }

  for (uint8_t i = 0; i < OUT_PINS_CNT; i++) {
    pinMode(out_pins[i], INPUT);  //high impedance
  }
  for (uint8_t i = 0; i < SCANNING_PINS_CNT; i++) {
    pinMode(scanning_pins[i], INPUT);
  }
}

void loop() {
  if (Serial.available() > 0) {
    char ch = Serial.read();
    Serial.print(ch);
    if (ch == '#') {
      Serial.write('#');
      return;  // skip rest of the loop for this iteration
    }
    if (ch == 'Y') {
      Serial.println("Manual keyboard test triggered.");
      keyboard_connection();
      for (uint8_t i = 0; i < OUT_PINS_CNT; i++) pinMode(out_pins[i], INPUT);
      for (uint8_t i = 0; i < SCANNING_PINS_CNT; i++) pinMode(scanning_pins[i], INPUT);
      c_pos = { -1, -1 };
    } else {
      c_pos = find_in_matrix(ch);
    }
    Serial.print(" Pos: [");
    Serial.print(c_pos.col);
    Serial.print(",");
    Serial.print(c_pos.row);
    Serial.println("]");
  }

  for (uint8_t i = 0; i < OUT_PINS_CNT; i++) {

    bool make_low = false;

    for (uint8_t j = 0; j < SCANNING_PINS_CNT; j++) {
      uint8_t state = 0;
      state = digitalRead(scanning_pins[j]);
      if (state == LOW) {

        if (c_pos.col == i && c_pos.row == j) {
          // Serial.print(c_pos.col);
          // Serial.print(c_pos.row);
          // digitalWrite(5, HIGH);
          make_low = true;
        }
      }
    }

    if (make_low) {
      pinMode(out_pins[i], OUTPUT);
      digitalWrite(out_pins[i], LOW);
    } else {
      pinMode(out_pins[i], INPUT);
    }
  }
}