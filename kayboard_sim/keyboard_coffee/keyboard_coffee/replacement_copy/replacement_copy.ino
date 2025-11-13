#include <Arduino.h>
#define COLS 5
#define ROWS 4

char matrix[ROWS][COLS] = {
  // 0    1    2   3   4  
  { '+', 'F', 'G','7','8' },  // 0
  { '-', 'B', 'C','3','4' },  // 1
  { ' ', '9', 'A','1','2' },  // 2
  { ' ', 'D', 'E','5','6' }   // 3
};

struct Position {
  int8_t row;
  int8_t col;
};

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

#define OUT_PINS_CNT 5
static uint8_t out_pins[OUT_PINS_CNT] = { 2, 3, 4,5,6};


#define SCANNING_PINS_CNT 4
static uint8_t scanning_pins[SCANNING_PINS_CNT] = { 7,8,9,10};

// #define SCANNING_PINS_CNT 1
// static uint8_t scanning_pins[SCANNING_PINS_CNT] = { 19};


void setup() {
  Serial.begin(9600);
  for (uint8_t i = 0; i < OUT_PINS_CNT; i++) {
    pinMode(out_pins[i], INPUT);  //high impedance
  }
  for (uint8_t i = 0; i < SCANNING_PINS_CNT; i++) {
    pinMode(scanning_pins[i], INPUT);
  }
}
Position c_pos;

void loop() {
  if (Serial.available() > 0) {
    char ch = Serial.read();
    Serial.print(ch);
    c_pos = find_in_matrix(ch);
    // Serial.print(c_pos.col);
    // Serial.print(c_pos.row);
  }

  //digitalWrite(5, digitalRead(19));

  for (uint8_t i = 0; i < OUT_PINS_CNT; i++) {

    bool make_low = false;

    for (uint8_t j = 0; j < SCANNING_PINS_CNT; j++) {
      uint8_t state = 0;
      state = digitalRead(scanning_pins[j]);


      // Serial.print("aaa");
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