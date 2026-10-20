/* EMF Linear Scanner firmware: keyboard-driven motion + live data stream.

   Keys (sent one character at a time by emf_scan.py from the graph window):
     D  move FORWARD until stopped        A  move BACKWARD until stopped
     W  faster (+100 steps/s)             S  slower (-100 steps/s)
     SPACE  stop the motor
     C  calibrate: moves BACK until the START switch (D12) is pressed = ZERO,
                   then FORWARD until the END switch (D11) is pressed = the EDGE.
                   SPACE sets either end by hand instead. Press C again to cancel.
     V  go to zero (needs a zero from calibration)
     .  nudge forward 1 mm        ,  nudge back 1 mm
     g<steps>  go to an absolute position, e.g. "g533" + newline = 10 mm (used by emf_scan.py
               for steps of 5, 10, 15... mm; positions are measured from zero)
     p  heartbeat (sent automatically by the computer, see WATCHDOG_MS)

   Output (115200 baud), one line per sample, every SAMPLE_INTERVAL_MS:
     time_ms,position_mm,sensor_V,field,mode,dir,step_rate,start_switch,end_switch
   (the two switch values are 1 = pressed, 0 = open, -1 = switches turned off)
   mode: 0 idle, 1 moving, 2 calibrating-back, 3 calibrating-forward, 4 going to zero
   speed is in steps per second (53.333 steps = 1 mm)
   Lines starting with '#' are status messages for the person at the keyboard.

   Works from the Arduino Serial Monitor too (the data lines just scroll past).

   Safety: once emf_scan.py is attached, the motor stops by itself if the computer goes quiet for WATCHDOG_MS,
   whenever a limit switch is pressed (START switch D12 going back, END switch D11 going forward),
   at the zero and the edge once calibrated, and after MAX_FREE_TRAVEL_MM of uncalibrated travel.
   A switch hit also re-syncs the position, so lost steps never build up.
*/
#include <Arduino.h>

// --- Hardware pins ---
const int EN = 8;
const int X_DIR = 2;
const int X_STEP = 5;
const int ZERO_SWITCH = 12;   // limit switch at the START of the rail, pressed = LOW (INPUT_PULLUP)
const int FAR_SWITCH = 11;    // limit switch at the far END of the rail, pressed = LOW (INPUT_PULLUP)
const int PROBE_PIN = A0;     // gaussmeter analog output (signal jack)
const int BIAS_PIN = A1;      // 2.5V bias node the gaussmeter's ground jack is tied to

// --- Motion settings ---
#ifndef USE_SWITCHES_DEFAULT
#define USE_SWITCHES_DEFAULT true
#endif
const bool USE_SWITCHES = USE_SWITCHES_DEFAULT;  // the two edge switches; set false if they are not connected
const float STEPS_PER_MM = 53.333;           // 1/16 microstepping, 30-tooth GT2 pulley
const long STEPS_PER_NUDGE = 53;             // about 1 mm
const unsigned int MIN_STEP_RATE = 100;      // steps per second
const unsigned int MAX_STEP_RATE = 2000;
const unsigned int STEP_RATE_CHANGE = 100;
unsigned int stepRate = 1000;                // about 18.8 mm/s. Lower it if the motor stalls
const unsigned long START_PERIOD_US = 4000;  // every move starts slowly...
const unsigned long RAMP_US = 40;            // ...and speeds up by this much each step
const unsigned long WATCHDOG_MS = 1500;      // stop if the computer goes quiet this long (only once emf_scan.py is attached)
const float MAX_FREE_TRAVEL_MM = 1500.0;     // longest uncalibrated move
const float MIN_RANGE_MM = 20.0;             // the edge must be at least this far from zero

// --- Probe calibration (SET THESE from your gaussmeter) ---
const float SENSOR_ZERO_V = 0.0;             // leave 0; zero the meter with its offset knob
// Output is +/-2V at full scale of the range switch:
//   +/-2 mT -> 1.0  |  +/-20 mT -> 10.0  |  +/-200 mT -> 100.0  |  +/-2000 mT -> 1000.0
const float FIELD_UNITS_PER_VOLT = 10.0;     // mT per volt. MUST match the meter's range

const unsigned long SAMPLE_INTERVAL_MS = 100;

// --- State ---
enum Mode { IDLE = 0, MOVING = 1, CAL_BACK = 2, CAL_FWD = 3, GOTO_ZERO = 4 };
Mode mode = IDLE;
int dir = 0;                      // +1 forward, -1 backward, 0 stopped
bool hasTarget = false;
long targetSteps = 0;
unsigned long curPeriod = START_PERIOD_US;
unsigned long lastStepUs = 0;
long positionSteps = 0;           // 0 at power-up, or at the zero point after calibration
long moveStartSteps = 0;
bool zeroSet = false;
bool calibrated = false;
long edgeSteps = 0;
unsigned long lastRxMs = 0;
bool hostSeen = false;             // set when emf_scan.py's heartbeat arrives
unsigned long lastSampleMs = 0;
bool gotoParsing = false;          // reading the digits of a "g<steps>" command
bool gotoNeg = false, gotoAny = false;
long gotoVal = 0;

// Differential reading: signal (A0) minus the bias node (A1), so negative
// voltages from the meter come out negative. Averages n readings.
float readSensorVolts(int n) {
  long sig = 0, bias = 0;
  for (int i = 0; i < n; i++) {
    sig += analogRead(PROBE_PIN);
    bias += analogRead(BIAS_PIN);
  }
  return ((sig - bias) / (float)n) * (5.0 / 1023.0);
}

// A switch counts as pressed only if two readings agree (ignores contact bounce / noise).
bool pressed(int pin) {
  if (digitalRead(pin) != LOW) return false;
  delayMicroseconds(50);
  return digitalRead(pin) == LOW;
}

void printSample() {
  float v = readSensorVolts(dir != 0 ? 4 : 16);   // fewer readings while stepping
  Serial.print(millis());                                      Serial.print(',');
  Serial.print((float)positionSteps / STEPS_PER_MM, 3);        Serial.print(',');
  Serial.print(v, 4);                                          Serial.print(',');
  Serial.print((v - SENSOR_ZERO_V) * FIELD_UNITS_PER_VOLT, 4); Serial.print(',');
  Serial.print((int)mode);                                     Serial.print(',');
  Serial.print(dir);                                           Serial.print(',');
  Serial.print(stepRate);                                      Serial.print(',');
  Serial.print(USE_SWITCHES ? (pressed(ZERO_SWITCH) ? 1 : 0) : -1);  Serial.print(',');
  Serial.println(USE_SWITCHES ? (pressed(FAR_SWITCH) ? 1 : 0) : -1);
}

void sampleIfDue() {
  unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    printSample();
  }
}

void stopMotor() {
  dir = 0;
  hasTarget = false;
  curPeriod = START_PERIOD_US;
  if (mode == MOVING || mode == GOTO_ZERO) mode = IDLE;
}

// Returns false (and prints why) if the move is not allowed.
bool startMove(int d, bool withTarget, long target) {
  if (d > 0 && calibrated && positionSteps >= edgeSteps) {
    Serial.println(F("#Already at the far edge"));
    return false;
  }
  if (d < 0 && calibrated && positionSteps <= 0) {
    Serial.println(F("#Already at zero"));
    return false;
  }
  if (d < 0 && USE_SWITCHES && pressed(ZERO_SWITCH)) {
    Serial.println(F("#START switch is pressed - cannot go further back"));
    return false;
  }
  if (d > 0 && USE_SWITCHES && pressed(FAR_SWITCH)) {
    Serial.println(F("#END switch is pressed - cannot go further forward"));
    return false;
  }
  if (dir != d) {
    digitalWrite(X_DIR, d > 0 ? HIGH : LOW);
    delayMicroseconds(20);
    curPeriod = START_PERIOD_US;
    lastStepUs = micros();
  }
  dir = d;
  hasTarget = withTarget;
  targetSteps = target;
  moveStartSteps = positionSteps;
  return true;
}

// The START switch was pressed: this is position 0. During calibration, carry on forward.
void atZeroSwitch() {
  Mode m = mode;
  stopMotor();                       // leaves the calibration mode untouched
  positionSteps = 0;
  zeroSet = true;
  if (m == CAL_BACK) {
    mode = CAL_FWD;
    Serial.println(F("#START switch found - zero set. Now moving FORWARD to the END switch"));
    startMove(1, false, 0);
  } else {
    Serial.println(F("#START switch hit - stopped (position set to 0)"));
  }
}

// The END switch was pressed. During calibration this is the far edge of the rail.
void atFarSwitch() {
  Mode m = mode;
  stopMotor();
  if (m == CAL_FWD) {
    if (positionSteps < (long)(MIN_RANGE_MM * STEPS_PER_MM)) {
      Serial.println(F("#END switch pressed very close to zero - check the switch wiring"));
      return;
    }
    edgeSteps = positionSteps;
    calibrated = true;
    mode = IDLE;
    Serial.print(F("#RANGE,0,"));
    Serial.print((float)edgeSteps / STEPS_PER_MM, 3);
    Serial.println(F(",switch"));
    Serial.println(F("#Calibration done - both limit switches found"));
  } else {
    if (calibrated) positionSteps = edgeSteps;    // the switch is the truth: re-sync
    Serial.println(F("#END switch hit - stopped"));
  }
}

// Called as often as possible; takes at most one step per call.
void runMotor() {
  if (dir == 0) return;
  unsigned long now = micros();
  if (now - lastStepUs < curPeriod) return;

  if (dir < 0) {
    if (USE_SWITCHES && pressed(ZERO_SWITCH)) {
      atZeroSwitch();
      return;
    }
    if (calibrated && positionSteps <= 0) {
      stopMotor();
      Serial.println(F("#Reached zero - stopped"));
      return;
    }
  } else {
    if (USE_SWITCHES && pressed(FAR_SWITCH)) {
      atFarSwitch();
      return;
    }
    if (calibrated && positionSteps >= edgeSteps) {
      stopMotor();
      Serial.println(F("#Reached the far edge - stopped"));
      return;
    }
  }
  if (!calibrated && labs(positionSteps - moveStartSteps) > (long)(MAX_FREE_TRAVEL_MM * STEPS_PER_MM)) {
    stopMotor();
    Serial.println(F("#Maximum travel reached - stopped"));
    return;
  }

  lastStepUs = now;
  digitalWrite(X_STEP, HIGH);
  delayMicroseconds(5);
  digitalWrite(X_STEP, LOW);
  positionSteps += dir;

  unsigned long goal = 1000000UL / stepRate;
  if (curPeriod > goal) {
    curPeriod = (curPeriod - goal > RAMP_US) ? curPeriod - RAMP_US : goal;
  } else {
    curPeriod = goal;
  }

  if (hasTarget && positionSteps == targetSteps) {
    bool toZero = (mode == GOTO_ZERO);
    stopMotor();
    if (toZero) Serial.println(F("#Arrived at zero"));
  }
}

// Go to an absolute position (in steps from zero), keeping inside the calibrated range.
void gotoSteps(long target) {
  if (mode == CAL_BACK || mode == CAL_FWD) {
    Serial.println(F("#Finish or cancel calibration first (C)"));
    return;
  }
  if (calibrated) {
    if (target < 0) target = 0;
    if (target > edgeSteps) target = edgeSteps;
  }
  if (target == positionSteps) {
    stopMotor();
    mode = IDLE;
    return;
  }
  mode = MOVING;
  if (!startMove(target > positionSteps ? 1 : -1, true, target)) stopMotor();
}

void printSpeed() {
  Serial.print(F("#Speed "));
  Serial.print(stepRate);
  Serial.print(F(" steps/s ("));
  Serial.print(stepRate / STEPS_PER_MM, 1);
  Serial.println(F(" mm/s)"));
}

void handleKey(char c) {
  switch (c) {
    case 'd':
    case 'a': {
      int d = (c == 'd') ? 1 : -1;
      if (mode == GOTO_ZERO) mode = IDLE;
      if (mode == IDLE) mode = MOVING;
      if (!startMove(d, false, 0)) stopMotor();
      break;
    }

    case ' ':
      if (mode == CAL_BACK) {
        stopMotor();
        positionSteps = 0;
        zeroSet = true;
        mode = CAL_FWD;
        Serial.println(F("#ZERO set by hand. Now moving FORWARD - press SPACE at the far edge (or let the END switch find it)"));
        startMove(1, false, 0);
      } else if (mode == CAL_FWD) {
        stopMotor();
        if (positionSteps < (long)(MIN_RANGE_MM * STEPS_PER_MM)) {
          Serial.println(F("#Edge is too close to zero - keep moving forward (D), then press SPACE"));
        } else {
          edgeSteps = positionSteps;
          calibrated = true;
          mode = IDLE;
          Serial.print(F("#RANGE,0,"));
          Serial.print((float)edgeSteps / STEPS_PER_MM, 3);
          Serial.println(F(",manual"));
          Serial.println(F("#Calibration done"));
        }
      } else {
        stopMotor();
        mode = IDLE;
        Serial.println(F("#Stopped"));
      }
      break;

    case 'c':
      stopMotor();
      if (mode == CAL_BACK || mode == CAL_FWD) {
        mode = IDLE;
        Serial.println(F("#Calibration cancelled"));
      } else {
        calibrated = false;
        zeroSet = false;
        mode = CAL_BACK;
        if (USE_SWITCHES && pressed(ZERO_SWITCH)) {
          atZeroSwitch();            // already sitting on the START switch: zero is here, go forward
        } else {
          Serial.println(F("#CALIBRATING: moving BACK to the START switch (or press SPACE at the zero position)"));
          startMove(-1, false, 0);
        }
      }
      break;

    case 'v':
      if (mode == CAL_BACK || mode == CAL_FWD) {
        Serial.println(F("#Finish or cancel calibration first (C)"));
      } else if (!zeroSet) {
        Serial.println(F("#No zero yet - run the calibration first (C)"));
      } else if (positionSteps == 0) {
        stopMotor();
        mode = IDLE;
        Serial.println(F("#Already at zero"));
      } else {
        mode = GOTO_ZERO;
        if (!startMove(positionSteps > 0 ? -1 : 1, true, 0)) stopMotor();
      }
      break;

    case 'w':
      if (stepRate < MAX_STEP_RATE) stepRate += STEP_RATE_CHANGE;
      printSpeed();
      break;
    case 's':
      if (stepRate > MIN_STEP_RATE) stepRate -= STEP_RATE_CHANGE;
      printSpeed();
      break;

    case '.':
    case ',': {
      int d = (c == '.') ? 1 : -1;
      if (mode == GOTO_ZERO) mode = IDLE;
      if (mode == IDLE) mode = MOVING;
      if (!startMove(d, true, positionSteps + d * STEPS_PER_NUDGE)) stopMotor();
      break;
    }

    case 'p':   // heartbeat from emf_scan.py: from now on the watchdog is active
      hostSeen = true;
      break;

    default:    // newlines and anything else are ignored
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(EN, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_STEP, OUTPUT);
  pinMode(ZERO_SWITCH, INPUT_PULLUP);
  pinMode(FAR_SWITCH, INPUT_PULLUP);
  digitalWrite(EN, LOW);
  lastRxMs = millis();
  Serial.println(F("#READY - EMF scanner firmware (goto, switches)"));
  if (USE_SWITCHES) {
    Serial.print(F("#Limit switches: START(D12) "));
    Serial.print(pressed(ZERO_SWITCH) ? F("PRESSED") : F("open"));
    Serial.print(F(", END(D11) "));
    Serial.println(pressed(FAR_SWITCH) ? F("PRESSED") : F("open"));
    if (pressed(ZERO_SWITCH) && pressed(FAR_SWITCH))
      Serial.println(F("#WARNING: both switches read pressed - check the wiring (pressed must pull the pin to GND)"));
  }
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    lastRxMs = millis();
    if (gotoParsing) {                       // digits after 'g' up to the newline
      if (c >= '0' && c <= '9' && gotoVal < 100000000L) {
        gotoVal = gotoVal * 10 + (c - '0');
        gotoAny = true;
      } else if (c == '-' && !gotoAny) {
        gotoNeg = true;
      } else {
        gotoParsing = false;
        if ((c == '\n' || c == '\r') && gotoAny) gotoSteps(gotoNeg ? -gotoVal : gotoVal);
      }
      continue;
    }
    if (c >= 'A' && c <= 'Z') c = c + 32;
    if (c == 'g') {
      gotoParsing = true; gotoNeg = false; gotoAny = false; gotoVal = 0;
    } else {
      handleKey(c);
    }
  }

  runMotor();

  if (hostSeen && dir != 0 && millis() - lastRxMs > WATCHDOG_MS) {
    stopMotor();
    Serial.println(F("#WATCHDOG: no signal from the computer - motor stopped"));
  }

  sampleIfDue();
}