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

enum BoomID {
    LEFT,
    RIGHT,
    CENTER,
};

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

void pump_on() {
    if (lowtank_warnings < lowtank_limit) {
      digitalWrite(A1, HIGH); // pump is connected to pin A1
      pump_state = true; 
      send_response(OK, PUMP, "Turned on");
    } else {
      send_response(ERROR, PUMP, "Water level too low");
    } 
}

void pump_off() {
  digitalWrite(A1, LOW);
  pump_state = false;
  send_response(OK, PUMP, "Turned off");
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

void spot_command(BoomID boomID, unsigned int nozzle, unsigned int duty) {
    if (!(
        boomID == CENTER // only support center boom for now
        && nozzle < 4 // only support 4 nozzles for now
        && duty <= MAX_DUTY_CYCLE
    )) {
        send_response(ERROR, SYSTEM, "Internal error while handling spot command");
        return;
    }
    
    if (duty == 0) {
      mcp.digitalWrite(nozzle, LOW);
    } else if (pump_state == false) { // fine to turn off nozzles with pump off
      send_response(ERROR, PUMP, "Pump off, cannot send nozzle command");
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

void handle_command(const char* const cmd, size_t read_len) {
    if (!(cmd != nullptr) || !(cmd[read_len] == '\0')) {
        send_response(ERROR, SYSTEM, "Internal error while parsing command");
        return;
    }
    if (read_len == 0) return;

    switch (cmd[0]) {
        case 'N':
            switch (cmd[1]) {
                case 'X': {
                    if (cmd[2] != '\0') goto invalid_command;
                    poweroff_command();
                    break;
                } // case 'X'
                case 'S':
                    switch (cmd[2]) {
                        case 'C': {
                            unsigned char nozzle_num = (unsigned char) cmd[3] - '0';
                            if (nozzle_num > 3) goto invalid_command;

                            unsigned int nozzle_state;
                            if (cmd[5] == '\0') { // backwards compatibility with older format (0/1)
                                unsigned char old_nozzle_state = (unsigned char) cmd[4] - '0';
                                
                                if (old_nozzle_state > 1)
                                    goto invalid_command;
                                
                                send_response(STATUS, SYSTEM, "Received old spot command");
                                nozzle_state = old_nozzle_state * MAX_DUTY_CYCLE;
                                
                            } else if (cmd[6] == '\0') { // check only two digits
                                unsigned int digit_first = (unsigned int) cmd[4] - '0';
                                unsigned int digit_second = (unsigned int) cmd[5] - '0';

                                if (digit_first > 9 || digit_second > 9)
                                    goto invalid_command;
                                
                                nozzle_state = digit_first * 10 + digit_second;
                                
                                if (nozzle_state > MAX_DUTY_CYCLE)
                                    goto invalid_command;
                                
                            } else { // more than two digits
                                goto invalid_command;
                            }
                            spot_command(CENTER, nozzle_num, nozzle_state);
                            
                            break;
                        } // case 'C'
                        case 'L': // left and right spray booms
                        case 'R':
                            goto not_implemented_command;
                        default:
                            goto invalid_command;
                    }
                    break;
                case 'B': // broadcast sprayer, not implemented
                    goto not_implemented_command;
                default:
                    goto invalid_command;
            }
            break;
        case 'P': {
            unsigned char status = (unsigned char) cmd[1] - '0';
            
            if (status == 0) {
                pump_off();
            } else if (status == 1) {
                pump_on();
            } else {
                goto invalid_command;
            }
            
            break;
        } // case 'P'
        default:
            goto invalid_command;
    }

    return;

    not_implemented_command:
    send_response(ERROR, SYSTEM, "Command not implemented");
    return;
    
    invalid_command:
    send_response(ERROR, SYSTEM, "Invalid serial command received");
    return;
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
  
    handle_command(buffer, bytesRead);
  }
  increment_pwm_nozzles(); 
  tanklevel_check();
}
