#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------- WATER RELAY ----------
const int RELAY_PIN = 13;  // Arduino digital pin, not PCA9685 channel 13
const bool RELAY_ACTIVE_LOW = true;
const int WATER_BELOW_PERCENT = 40;

// ---------- STEPPER AND HOME SWITCH ----------
const int dirPin  = 2;
const int stepPin = 3;
const int homePin = 4;  // NO switch: D4 to NO, GND to COM

const int ULTRASONIC_1_TRIG = 7;
const int ULTRASONIC_1_ECHO = 8;
const int ULTRASONIC_2_TRIG = 9;
const int ULTRASONIC_2_ECHO = 10;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;

const long MICROSTEPS    = 32;
const long STEPS_PER_REV = 200L * MICROSTEPS;

const float MM_PER_REV   = 20.0;
const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;

// Geometry from the home-switch/gantry-edge reference.
// Assumes the arm probe is 90 mm toward the stations from the edge
// that contacts the home switch.
const float ARM_OFFSET_FROM_EDGE_MM = 90.0;
const float EDGE_TO_STATION1_MM = 140.0;
const float STATION_SPACING_MM = 140.0;

const float STATION_1_GANTRY_MM =
    EDGE_TO_STATION1_MM - ARM_OFFSET_FROM_EDGE_MM;  // 50 mm
const float STATION_2_GANTRY_MM =
    STATION_1_GANTRY_MM + STATION_SPACING_MM;       // 190 mm
const float STATION_3_GANTRY_MM =
    STATION_2_GANTRY_MM + STATION_SPACING_MM;       // 330 mm

const long STATION_1_POSITION =
    (long)(STATION_1_GANTRY_MM * STEPS_PER_MM + 0.5);
const long STATION_2_POSITION =
    (long)(STATION_2_GANTRY_MM * STEPS_PER_MM + 0.5);
const long STATION_3_POSITION =
    (long)(STATION_3_GANTRY_MM * STEPS_PER_MM + 0.5);

// Must exceed 330 mm so homing can find the switch from station 3.
// Confirm 620 mm is within the rail's safe travel if the switch fails.
const float MAX_HOMING_TRAVEL_MM = 350.0;
const long HOMING_MAX_STEPS =
    (long)(MAX_HOMING_TRAVEL_MM * STEPS_PER_MM + 0.5);

// Position is tracked from the switch trigger point.
long currentPositionSteps = 0;

// Normal travel pulse delays; each is half of a STEP pulse.
const int startDelay  = 900;
const int targetDelay = 400;
const long rampSteps  = 4800;

// Slower seek and final approach for homing.
const int HOME_FAST_DELAY = 1000;
const int HOME_SLOW_DELAY = 2000;
const long HOME_CLEARANCE_STEPS =
    (long)(3.0 * STEPS_PER_MM + 0.5);

const int DIR_FORWARD  = LOW;
const int DIR_BACKWARD = HIGH;

// Assumes backward motion goes toward the home switch.
const int DIR_HOME = DIR_BACKWARD;
const int DIR_AWAY = DIR_FORWARD;

const unsigned long HOME_PAUSE_MS = 20000;

// ---------- PCA9685 SERVO CHANNELS ----------
// These are PCA9685 channel numbers, not Arduino pin numbers.
const int BASE     = 0;
const int SHOULDER = 1;
const int ELBOW    = 2;
const int WRIST    = 3;
const int SERVO_CHANNELS[] = {12, 13, 14, 15};

// ---------- ARM LIMITS ----------
const int limitsMin[] = {0, 0, 30, 0};
const int limitsMax[] = {180, 150, 150, 90};

//                         base  shoulder  elbow  wrist
const int START[]       = {   0,      80,     90,    90};
const int DIR[]         = {  +1,      +1,     +1,    -1};
const int TRAVEL[]      = { 165,      30,     30,    45};

int POINT_B[4];

const float SPEED[] = {180, 100, 60, 180};
const float MIN_TIME = 0.2;
const int PAUSE_MS = 100;
const int HOLD_TIME = 10;

//                         base  shoulder  elbow  wrist
const int PULSE_MIN[] = {  375,     600,     600,  600};
const int PULSE_MAX[] = { 2625,    2400,    2400, 2400};
float currentPos[] = {0, 80, 90, 90};

// ---------- MOISTURE SENSOR ----------
const int MOISTURE_PIN = A0;
const unsigned long READ_INTERVAL = 1000;
const int DRY_VALUE = 520;
const int WET_VALUE = 260;

unsigned long lastReadTime = 0;
int moisturePercent = 0;

// ---------- NON-BLOCKING ULTRASONIC DISTANCE READINGS ----------
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  bool waitingForRise;
  bool waitingForFall;
  unsigned long pingStartUs;
  unsigned long echoRiseUs;
  float distanceCm;
};

UltrasonicSensor ultrasonicSensors[2] = {
  {ULTRASONIC_1_TRIG, ULTRASONIC_1_ECHO, false, false, 0, 0, -1.0},
  {ULTRASONIC_2_TRIG, ULTRASONIC_2_ECHO, false, false, 0, 0, -1.0}
};

const unsigned long ULTRASONIC_PING_GAP_US = 60000UL;
const unsigned long ULTRASONIC_REPORT_INTERVAL_MS = 500UL;
int activeUltrasonicSensor = 0;
unsigned long nextUltrasonicPingUs = 0;
unsigned long lastUltrasonicReportMs = 0;

void finishUltrasonicPing(unsigned long nowUs) {
  UltrasonicSensor &sensor = ultrasonicSensors[activeUltrasonicSensor];
  sensor.waitingForRise = false;
  sensor.waitingForFall = false;
  activeUltrasonicSensor = 1 - activeUltrasonicSensor;
  nextUltrasonicPingUs = nowUs + ULTRASONIC_PING_GAP_US;
}

void printUltrasonicDistance(int sensorNumber, float distanceCm) {
  Serial.print("US");
  Serial.print(sensorNumber);
  Serial.print("=");
  if (distanceCm < 0) {
    Serial.print("no echo");
  } else {
    Serial.print(distanceCm, 1);
    Serial.print(" cm");
  }
}

void serviceUltrasonicSensors() {
  unsigned long nowUs = micros();
  UltrasonicSensor &sensor = ultrasonicSensors[activeUltrasonicSensor];

  if (sensor.waitingForRise || sensor.waitingForFall) {
    if (nowUs - sensor.pingStartUs >= ULTRASONIC_TIMEOUT_US) {
      sensor.distanceCm = -1.0;
      finishUltrasonicPing(nowUs);
    } else if (sensor.waitingForRise && digitalRead(sensor.echoPin) == HIGH) {
      sensor.echoRiseUs = nowUs;
      sensor.waitingForRise = false;
      sensor.waitingForFall = true;
    } else if (sensor.waitingForFall && digitalRead(sensor.echoPin) == LOW) {
      unsigned long echoDurationUs = nowUs - sensor.echoRiseUs;
      sensor.distanceCm = echoDurationUs * 0.0343 / 2.0;
      finishUltrasonicPing(nowUs);
    }
  } else if ((long)(nowUs - nextUltrasonicPingUs) >= 0) {
    digitalWrite(sensor.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.trigPin, LOW);

    sensor.pingStartUs = micros();
    sensor.waitingForRise = true;
  }

  unsigned long nowMs = millis();
  if (nowMs - lastUltrasonicReportMs >= ULTRASONIC_REPORT_INTERVAL_MS) {
    lastUltrasonicReportMs = nowMs;
    printUltrasonicDistance(1, ultrasonicSensors[0].distanceCm);
    Serial.print("  ");
    printUltrasonicDistance(2, ultrasonicSensors[1].distanceCm);
    Serial.println();
  }
}

// ---------- STATUS AND RELAY ----------
void showStatus(const char* text) {
  Serial.println(text);
}

void pumpOn() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
}

void pumpOff() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
}

// ---------- MOISTURE READING ----------
void readMoistureNow() {
  long total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(MOISTURE_PIN);
    delay(1);
  }

  int raw = total / 5;
  moisturePercent =
      constrain(map(raw, DRY_VALUE, WET_VALUE, 0, 100), 0, 100);
  lastReadTime = millis();

  Serial.print("Moisture raw: ");
  Serial.print(raw);
  Serial.print(" | ");
  Serial.print(moisturePercent);
  Serial.println("%");
}

void updateMoisture() {
  if (millis() - lastReadTime >= READ_INTERVAL) {
    readMoistureNow();
  }
}

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateMoisture();
    serviceUltrasonicSensors();
  }
}

// ---------- PROBE, READ, AND WATER IF NEEDED ----------
void holdAtB() {
  // Take a fresh reading while the probe is at the soil.
  readMoistureNow();

  bool watering = (moisturePercent < WATER_BELOW_PERCENT);
  if (watering) {
    pumpOn();
    Serial.println("Below threshold; watering.");
  } else {
    Serial.println("Above threshold; holding.");
  }

  for (int seconds = HOLD_TIME; seconds > 0; seconds--) {
    Serial.print("Hold time remaining: ");
    Serial.println(seconds);
    waitMs(1000);
  }

  pumpOff();
}

// ---------- ARM SERVO CONTROL ----------
void setAngle(int joint, float angle) {
  angle = constrain(angle, limitsMin[joint], limitsMax[joint]);
  currentPos[joint] = angle;

  int pulse = PULSE_MIN[joint] +
      (int)(angle * (PULSE_MAX[joint] - PULSE_MIN[joint]) / 180.0);

  pwm.writeMicroseconds(SERVO_CHANNELS[joint], pulse);
}

float ease(float t) {
  return t * t * (3.0 - 2.0 * t);
}

void moveJoint(int joint, float target) {
  target = constrain(target, limitsMin[joint], limitsMax[joint]);

  float startAngle = currentPos[joint];
  float distance = target - startAngle;
  if (distance < 0) {
    distance = -distance;
  }

  if (distance < 0.5) {
    return;
  }

  float duration = distance / SPEED[joint];
  if (duration < MIN_TIME) {
    duration = MIN_TIME;
  }

  int steps = (int)(duration / 0.02);
  if (steps < 6) {
    steps = 6;
  }

  for (int s = 1; s <= steps; s++) {
    float progress = ease((float)s / steps);
    float angle = startAngle + (target - startAngle) * progress;
    setAngle(joint, angle);
    waitMs((unsigned long)(duration * 1000.0 / steps));
  }
}

void goToPointB() {
  showStatus("Elbow -> probe");
  moveJoint(ELBOW, POINT_B[ELBOW]);
  waitMs(PAUSE_MS);

  showStatus("Wrist -> probe");
  moveJoint(WRIST, POINT_B[WRIST]);
  waitMs(PAUSE_MS);

  showStatus("Shoulder -> probe");
  moveJoint(SHOULDER, POINT_B[SHOULDER]);
}

void goToStart() {
  showStatus("Shoulder -> start");
  moveJoint(SHOULDER, START[SHOULDER]);
  waitMs(PAUSE_MS);

  showStatus("Elbow -> start");
  moveJoint(ELBOW, START[ELBOW]);
  waitMs(PAUSE_MS);

  showStatus("Wrist -> start");
  moveJoint(WRIST, START[WRIST]);
  waitMs(PAUSE_MS);
}

void takeReading(int station, const char* side) {
  Serial.print("Station ");
  Serial.print(station);
  Serial.print(" - ");
  Serial.println(side);

  goToPointB();
  holdAtB();
  goToStart();
}

void inspectStation(int station) {
  // Read the first side with the base in its starting direction.
  moveJoint(BASE, START[BASE]);
  takeReading(station, "first side");

  // Turn the base 180 degrees and read the opposite side.
  showStatus("Base -> opposite");
  moveJoint(BASE, POINT_B[BASE]);
  waitMs(500);
  takeReading(station, "opposite side");

  // Face the original direction before the next rail move.
  showStatus("Base -> start side");
  moveJoint(BASE, START[BASE]);
  waitMs(500);
}

// ---------- STEPPER ----------
void stepPulse(int halfPulseDelay) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(halfPulseDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(halfPulseDelay);
  serviceUltrasonicSensors();
}

void moveSteps(long steps, int direction) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(10);

  for (long i = 0; i < steps; i++) {
    long fromEnd = i;
    long stepsToEnd = steps - 1 - i;
    if (stepsToEnd < fromEnd) {
      fromEnd = stepsToEnd;
    }

    int pulseDelay = targetDelay;
    if (fromEnd < rampSteps) {
      pulseDelay = startDelay -
          (long)(startDelay - targetDelay) * fromEnd / rampSteps;
    }

    stepPulse(pulseDelay);

    if (direction == DIR_FORWARD) {
      currentPositionSteps++;
    } else {
      currentPositionSteps--;
    }
  }

  digitalWrite(stepPin, LOW);
}

void moveToPosition(long targetPosition) {
  long difference = targetPosition - currentPositionSteps;

  if (difference > 0) {
    moveSteps(difference, DIR_FORWARD);
  } else if (difference < 0) {
    moveSteps(-difference, DIR_BACKWARD);
  }
}

// ---------- HOMING ----------
bool homeSwitchPressed() {
  // Normally open switch to GND: LOW means pressed.
  return digitalRead(homePin) == LOW;
}

bool moveUntilHomeState(
    bool wantPressed,
    int direction,
    long maxSteps,
    int halfPulseDelay
) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(10);

  for (long i = 0; i < maxSteps; i++) {
    if (homeSwitchPressed() == wantPressed) {
      delay(5);
      if (homeSwitchPressed() == wantPressed) {
        digitalWrite(stepPin, LOW);
        return true;
      }
    }

    stepPulse(halfPulseDelay);
  }

  digitalWrite(stepPin, LOW);
  return false;
}

void moveRawSteps(long steps, int direction, int halfPulseDelay) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(10);

  for (long i = 0; i < steps; i++) {
    stepPulse(halfPulseDelay);
  }

  digitalWrite(stepPin, LOW);
}

void homingError(const char* message) {
  digitalWrite(stepPin, LOW);
  pumpOff();
  Serial.print("HOMING ERROR: ");
  Serial.println(message);

  while (true) {
    delay(1000);
  }
}

// backOffAfterHome=true clears the switch after homing.
// false leaves the carriage at the switch for the end-of-cycle pause.
void homeGantry(bool backOffAfterHome) {
  showStatus("Homing gantry");

  if (homeSwitchPressed()) {
    if (!moveUntilHomeState(
            false, DIR_AWAY, HOME_CLEARANCE_STEPS * 5L, HOME_FAST_DELAY)) {
      homingError("Switch stuck?");
    }
  }

  // Fast seek toward home.
  if (!moveUntilHomeState(
          true, DIR_HOME, HOMING_MAX_STEPS, HOME_FAST_DELAY)) {
    homingError("Switch not found");
  }

  // Back off until released, then clear it farther.
  if (!moveUntilHomeState(
          false, DIR_AWAY, HOME_CLEARANCE_STEPS * 5L, HOME_FAST_DELAY)) {
    homingError("Switch did not release");
  }
  moveRawSteps(HOME_CLEARANCE_STEPS, DIR_AWAY, HOME_SLOW_DELAY);

  // Slow final approach establishes the repeatable zero.
  if (!moveUntilHomeState(
          true, DIR_HOME, HOME_CLEARANCE_STEPS * 5L, HOME_SLOW_DELAY)) {
    homingError("Slow home failed");
  }

  currentPositionSteps = 0L;

  if (backOffAfterHome) {
    moveRawSteps(HOME_CLEARANCE_STEPS, DIR_AWAY, HOME_SLOW_DELAY);
    currentPositionSteps = HOME_CLEARANCE_STEPS;
  }

  showStatus("Home set");
}

void pauseAtHome(unsigned long pauseMs) {
  digitalWrite(stepPin, LOW);
  Serial.print("At home; pausing ");
  Serial.print(pauseMs / 1000);
  Serial.println(" seconds.");
  waitMs(pauseMs);
}

// ---------- SETUP ----------
void setup() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(RELAY_PIN, OUTPUT);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(homePin, INPUT_PULLUP);
  pinMode(ULTRASONIC_1_TRIG, OUTPUT);
  pinMode(ULTRASONIC_1_ECHO, INPUT);
  pinMode(ULTRASONIC_2_TRIG, OUTPUT);
  pinMode(ULTRASONIC_2_ECHO, INPUT);
  digitalWrite(ULTRASONIC_1_TRIG, LOW);
  digitalWrite(ULTRASONIC_2_TRIG, LOW);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);

  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    POINT_B[i] = constrain(
        START[i] + DIR[i] * TRAVEL[i],
        limitsMin[i],
        limitsMax[i]
    );
  }

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  setAngle(SHOULDER, START[SHOULDER]);
  delay(300);
  setAngle(ELBOW, START[ELBOW]);
  delay(300);
  setAngle(WRIST, START[WRIST]);
  delay(300);
  setAngle(BASE, START[BASE]);
  delay(1000);

  // Home at startup, then clear the switch before starting the route.
  homeGantry(true);
}

// ---------- MAIN ROUTE: THREE POSITIONS, BOTH SIDES ----------
void loop() {
  Serial.println("Moving to station 1 midpoint: 50 mm from home.");
  moveToPosition(STATION_1_POSITION);
  inspectStation(1);

  Serial.println("Moving to station 2 midpoint: 190 mm from home.");
  moveToPosition(STATION_2_POSITION);
  inspectStation(2);

  Serial.println("Moving to station 3 midpoint: 330 mm from home.");
  moveToPosition(STATION_3_POSITION);
  inspectStation(3);

  Serial.println("Returning to the home switch.");
  homeGantry(false);
  pauseAtHome(HOME_PAUSE_MS);
}



