#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;
const int SIGNAL_FREQUENCY = 20; // 20 Hz
const int SIGNAL_PERIOD = 1000 / SIGNAL_FREQUENCY; // Hz to ms
const int NUM_SPOT = 4; // number of spot nozzles
// represents fully on, valid duty cycles to pass in range from 0 to MAX_DUTY_CYCLE
const int MAX_DUTY_CYCLE = 50;

char buffer[32];
unsigned long previousTime[NUM_SPOT] = {0};
unsigned int high_time[NUM_SPOT]; // pwm high duration in ms
unsigned int low_time[NUM_SPOT]; // pwm low duration in ms
int spot_pulse_state[NUM_SPOT]; // short term record of where solenoid is in PWM cycle
int spot_nozzle_state[NUM_SPOT]; // record of which spot solenoids are on/off

int lowtank_warnings = 0;
const int lowtank_limit = 3; // need three low tank warnings before turning pump off. 
unsigned long previous_tankcheck = 0;
bool pump_state = false;  

enum ResponseMessageStatus {
    OK,
    ERROR,
    STATUS,
};

enum ResponseMessageSource {
    SYSTEM, // for things like invalid commands, library checks
    SPOT,   // related to the spot sprayer
    PUMP,   // related to pump, including pump too low
};

void send_response(ResponseMessageStatus status, ResponseMessageSource source, const char* message) {
  // confirm message is valid (no newlines allowed)
  const char* curr = message;
  while (*curr != '\0') {
    if (*curr == '\n') {
      send_response(ERROR, SYSTEM, "invalid response attempted, message contains newline");
      return;
    }
    curr++;
  }

  switch (status) {
    case OK:
      Serial.print("OKAY ");
      break;
    case ERROR:
      Serial.print("ERRO ");
      break;
    case STATUS:
      Serial.print("STAT ");
      break;
  }

  switch (source) {
    case SYSTEM:
      Serial.print("(SYST): ");
      break;
    case SPOT:
      Serial.print("(SPOT): ");
      break;
    case PUMP:
      Serial.print("(PUMP): ");
      break;
  }

  Serial.print(message);
  Serial.println(); // must newline terminate messages so listener knows it's ended
}

void poweroff_command() {
  send_response(OK, SYSTEM, "Turning everything off");
  mcp.digitalWrite(0, LOW); 
  mcp.digitalWrite(1, LOW);
  mcp.digitalWrite(2, LOW); 
  mcp.digitalWrite(3, LOW); 
}

void pump_command() {
  int state = buffer[1] - '0';
  if (state == 1) {
    if (lowtank_warnings < lowtank_limit) {
      digitalWrite(A1, HIGH); // pump is connected to pin A1
      pump_state = true; 
      send_response(OK, PUMP, "Turned on");
    } else {
      send_response(ERROR, PUMP, "Water level too low");
    }
  } else if (state == 0) {
    digitalWrite(A1, LOW);
    pump_state = false;
    send_response(OK, PUMP, "Turned off");
  } else {
    send_response(ERROR, SYSTEM, "Invalid pump state");
  }
}

void tanklevel_check() {
  unsigned long current_time = millis();
  if (current_time - previous_tankcheck > 1000) {
    if (digitalRead(A3) == 0) {
      lowtank_warnings++;
    } else {
      lowtank_warnings = 0;
    }
    previous_tankcheck = current_time;
  }
  if (lowtank_warnings >= lowtank_limit && pump_state) {
    digitalWrite(A1, LOW);
    poweroff_command();
    pump_state = false;
    send_response(STATUS, SYSTEM, "Water level low, all nozzles shut down");
  } 
}

void spot_command() {
  char boomID = buffer[2]; // L(eft), R(ight), C(enter)
  int nozzle = buffer[3] - '0'; // 0,1,2,3
  int duty; 
  if (buffer[5] != '\0') { // duty cycle is two digits
    duty = (buffer[4] - '0')*10 + (buffer[5] - '0'); // [0, MAX_DUTY_CYCLE] duty
  } else {
    duty = buffer[4] - '0'; // backwards compatibility with 0/1 on-off commands
    duty *= MAX_DUTY_CYCLE; // convert to either 0 (off) or MAX_DUTY_CYCLE (fully on)
    send_response(STATUS, SYSTEM, "Received old spot command");
  }
  if (duty == 0) {
    mcp.digitalWrite(nozzle, LOW);
  } else if (duty > MAX_DUTY_CYCLE) {
    send_response(ERROR, SYSTEM, "Invalid duty cycle received for spot command");
    return;
  }
  spot_nozzle_state[nozzle] = duty;
  // really (duty / MAX_DUTY_CYCLE) * SIGNAL_PERIOD,
  // but integer math requires this ordering
  high_time[nozzle] = duty * SIGNAL_PERIOD / MAX_DUTY_CYCLE;
  low_time[nozzle] = SIGNAL_PERIOD - high_time[nozzle];
  send_response(OK, SPOT, "Set Spot Nozzle");
}

void increment_pwm_nozzles() {
  unsigned long currentTime = millis();
  for (int n = 0; n < NUM_SPOT; n++) {
    if (spot_nozzle_state[n] != 0) {
      if (spot_pulse_state[n] == 1 && (currentTime - previousTime[n] >= high_time[n])) {
        spot_pulse_state[n] = 0;
        previousTime[n] = currentTime;
        mcp.digitalWrite(n, LOW); 
      } else if (spot_pulse_state[n] == 0 && (currentTime - previousTime[n] >= low_time[n])) {
        spot_pulse_state[n] = 1;
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
    send_response(ERROR, SYSTEM, "Couldn't find MCP23017");
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
      pump_command();
    } else if (command == 'N') { // NOZZLE
      char nozzletype = buffer[1]; // S(Spot) or X(turn all off)
      if (nozzletype == 'X') { // X: all nozzles off command
        poweroff_command();
      } else if (nozzletype == 'S') { // S: spot spray command
        if (pump_state) {
          spot_command();
        } else {
          send_response(ERROR, PUMP, "Pump off, cannot send nozzle command");
        }
      } else {
        send_response(ERROR, SYSTEM, "Invalid nozzle command received");
      }
    } else {
        send_response(ERROR, SYSTEM, "Invalid serial command received");
    }
  }
  increment_pwm_nozzles(); 
  tanklevel_check();
}
