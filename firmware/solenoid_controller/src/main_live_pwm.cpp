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

void poweroff_command() {
  Serial.print("Turning everything off");
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
      Serial.println("ACK: Pump turned on.");
    } else {
      Serial.println("Water level low. Pump cannot be turned on.");
    }
  } else if (state == 0) {
    digitalWrite(A1, LOW);
    pump_state = false;
    Serial.println("ACK: Pump turned off.");
  } else {
    Serial.println("Invalid pump state received");
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
    Serial.println("Tank level low. Pump and all nozzles turned off.");
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
    Serial.println("ACK: Received old on-off spot command");
  }
  if (duty == 0) {
    mcp.digitalWrite(nozzle, LOW);
  } else if (duty > MAX_DUTY_CYCLE) {
    Serial.println("Invalid duty cycle received for spot command");
    return;
  }
  spot_nozzle_state[nozzle] = duty;
  // really (duty / MAX_DUTY_CYCLE) * SIGNAL_PERIOD,
  // but integer math requires this ordering
  high_time[nozzle] = duty * SIGNAL_PERIOD / MAX_DUTY_CYCLE;
  low_time[nozzle] = SIGNAL_PERIOD - high_time[nozzle];
  Serial.print("ACK: Set Spot Nozzle ");
  Serial.print(nozzle);
  Serial.print(" to ");
  Serial.println(duty);
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
      pump_command();
    } else if (command == 'N') { // NOZZLE
      char nozzletype = buffer[1]; // S(Spot) or X(turn all off)
      if (nozzletype == 'X') { // X: all nozzles off command
        poweroff_command();
      } else if (nozzletype == 'S') { // S: spot spray command
        if (pump_state) {
          spot_command();
        } else {
          Serial.println("Pump off due to low water level. Cannot send nozzle command.");
        }
      } else {
        Serial.println("Invalid nozzle command received");
      }
    } else {
      Serial.println("Invalid serial comamnd received");
    }
  }
  increment_pwm_nozzles(); 
  tanklevel_check();
}
