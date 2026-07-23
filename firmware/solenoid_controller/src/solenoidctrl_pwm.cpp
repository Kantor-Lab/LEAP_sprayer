#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;
const int SIGNAL_FREQUENCY = 20; // 20 Hz
const int SIGNAL_PERIOD = 50; // 1/20 Hz = 50 ms
const int NUM_BROADCAST = 4; // number of broadcast nozzles

char buffer[32];
unsigned long previousTime[NUM_BROADCAST] = {0};
unsigned int high_time[NUM_BROADCAST]; // pwm high duration in ms
unsigned int low_time[NUM_BROADCAST]; // pwm low duration in ms
int broadcast_pulse_state[NUM_BROADCAST]; // short term record of where solenoid is in PWM cycle
int broadcast_nozzle_state[NUM_BROADCAST]; // record of which broadcast solenoids are on/off


void poweroff_command() {
  Serial.print("Turning everything off");
  mcp.digitalWrite(0, LOW); 
  mcp.digitalWrite(1, LOW);
  mcp.digitalWrite(2, LOW); 
  mcp.digitalWrite(3, LOW); 
}

void spot_command() {
  char boomID = buffer[2]; // L(Left), R(Right), C(Center)
  int nozzle = buffer[3] - '0'; // 0,1,2,3 
  int state = buffer[4] - '0'; // 1: on || 0: off
  if (state == 1) {
    mcp.digitalWrite(nozzle, HIGH);
  } else if (state == 0) {
    mcp.digitalWrite(nozzle, LOW);
  } else {
    Serial.println("Invalid spot nozzle state received: ");
  }
  Serial.print("ACK: Set Spot Nozzle ");
  Serial.print(nozzle);
  Serial.print(" to ");
  Serial.println(state);
}

void broadcast_command() {
  int nozzle = buffer[2] - '0'; // 0 = Center, 1 = L, 2 = R
  int duty = (buffer[3] - '0')*10 + (buffer[4] - '0'); // [0, 100) duty
  if (duty == 0) {
    mcp.digitalWrite(nozzle, LOW);
  }
  broadcast_nozzle_state[nozzle] = duty;
  high_time[nozzle] = (duty * SIGNAL_PERIOD) * 0.01;
  low_time[nozzle] = SIGNAL_PERIOD - high_time[nozzle];
  Serial.print("ACK: Set Broadcast Nozzle ");
  Serial.print(nozzle);
  Serial.print(" to ");
  Serial.println(duty);
}

void increment_broadcastnozzles() {
  unsigned long currentTime = millis();
  for (int n = 0; n < NUM_BROADCAST; n++) {
    if (broadcast_nozzle_state[n] != 0) {
      if (broadcast_pulse_state[n] == 1 && (currentTime - previousTime[n] >= high_time[n])) {
        broadcast_pulse_state[n] = 0;
        previousTime[n] = currentTime;
        mcp.digitalWrite(n, LOW); 
      } else if (broadcast_pulse_state[n] == 0 && (currentTime - previousTime[n] >= low_time[n])) {
        broadcast_pulse_state[n] = 1;
        previousTime[n] = currentTime;
        mcp.digitalWrite(n, HIGH);
      }
    } 
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  if (!mcp.begin_I2C()) {
    Serial.println("Couldn't find MCP23017..");
    while (1);
  }
  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, OUTPUT);
  mcp.pinMode(2, OUTPUT);
  mcp.pinMode(3, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    int bytesRead = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[bytesRead] = '\0';
  
    char command = buffer[0]; // N(Nozzle), P(Pump)
    if (command == 'P') { // PUMP
      Serial.println("Pump command");
    } else if (command == 'N') { // NOZZLE
      char nozzletype = buffer[1]; // S(Spot) or B(Broadcast) or X(turn all off)
      if (nozzletype == 'X') { // X: all nozzles off command
        poweroff_command();
      } else if (nozzletype == 'S') { // S: spot spray command
        spot_command();
      } else if (nozzletype == 'B') { // B: broadcast spray command
        broadcast_command();
      } else {
        Serial.println("Invalid nozzle command received");
      }
    } else {
      Serial.println("Invalid serial comamnd received");
    }
  }
  increment_broadcastnozzles(); 
}
