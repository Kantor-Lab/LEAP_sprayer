#include <Arduino.h>

char buffer[32];

void poweroff_seq() {
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    int bytesRead = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[bytesRead] = '\0';

    char command = buffer[0]; // N(Nozzle)
    char nozzletype = buffer[1]; // S(Spot) or B(Broadcast)
    char boomID = buffer[2]; // L(Left), R(Right), C(Center)
    int nozzle = buffer[3] - '0'; // 0,1,2,3 
    int state = buffer[4] - '0'; // 1: on || 0: off
    if (command == 'N' && nozzletype == 'X') {
      poweroff_seq();
    } else if (command == 'N') {
      if (state == 1) {
        digitalWrite(nozzle+2, HIGH);
      } else if (state == 0) {
        digitalWrite(nozzle+2, LOW);
      } else {
        Serial.println("Invalid serial command received: ");
      }
      Serial.print("ACK: Set Nozzle ");
      Serial.print(nozzle);
      Serial.print(" to ");
      Serial.println(state);
    } else {
      Serial.println("Invalid serial command received: ");
    }
  }
}


