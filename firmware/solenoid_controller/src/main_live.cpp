#include <Adafruit_MCP23X17.h>
#define NOID_1 0
#define NOID_2 1
#define NOID_3 2
#define NOID_4 3

char buffer[32];
Adafruit_MCP23X17 mcp;

void poweroff_seq() {
  mcp.digitalWrite(NOID_1, LOW);
  mcp.digitalWrite(NOID_2, LOW);
  mcp.digitalWrite(NOID_3, LOW);
  mcp.digitalWrite(NOID_4, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  if (!mcp.begin_I2C()) {
    Serial.println("Couldn't find MCP23017..");
    while (1);
  }
  mcp.pinMode(NOID_1, OUTPUT);
  mcp.pinMode(NOID_2, OUTPUT);
  mcp.pinMode(NOID_3, OUTPUT);
  mcp.pinMode(NOID_4, OUTPUT);
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
        mcp.digitalWrite(nozzle, HIGH);
      } else if (state == 0) {
        mcp.digitalWrite(nozzle, LOW);
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

