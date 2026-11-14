
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

// ==================================================
// PIN CONFIGURATION
// ==================================================

// L298N
#define ENA 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ENB 11

// Left Sharp IR
#define IR_LEFT A1

// Finish line IR
#define IR_FINISH 5

// Change to HIGH if your sensor outputs HIGH on black
#define BLACK_DETECTED LOW

// MPU6050
#define MPU_ADDR 0x68

// VL53L0X
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

// ==================================================
// FINISH LINE SETTINGS
// ==================================================

const int FINISH_FORWARD_SPEED = 150;
const int FINISH_FORWARD_TIME = 300;

// ==================================================
// LEFT WALL PID SETTINGS
// ==================================================

const float LEFT_TARGET = 8.0;
const float DEADBAND = 2.0;

const int BASE_SPEED = 170;
const int MAX_CORRECTION = 120;

const float KP = 5.0;

int previousCorrection = 0;

// ==================================================
// LEFT OPENING DETECTION
// ==================================================

const float LEFT_OPEN_THRESHOLD = 15.0;
const float LEFT_REARM_DISTANCE = 11.0;

const int LEFT_OPEN_CONFIRM_COUNT = 2;

int leftOpenCount = 0;
bool leftOpeningArmed = true;

// ==================================================
// LEFT OPENING TURN SETTINGS
// ==================================================

const int LEFT_TURN_LEFT_SPEED = 50;
const int LEFT_TURN_RIGHT_SPEED = 150;

const float LEFT_WALL_FOUND_DISTANCE = 10.0;

const int WALL_FOUND_COUNT = 3;

// ==================================================
// FRONT OBSTACLE SETTINGS
// ==================================================

const int FRONT_THRESHOLD_MM = 100;
const int CONFIRM_COUNT = 2;

int frontCount = 0;
unsigned long ignoreFrontUntil = 0;

// ==================================================
// MOVEMENT BEFORE TURNS
// ==================================================

const int REVERSE_SPEED = 250;
const int REVERSE_TIME = 80;

// ==================================================
// SHARP IR CALIBRATION
// ==================================================

const int distances[] = {5, 8, 10, 15, 20, 25, 30};
const int readings[]  = {450, 300, 250, 170, 130, 100, 80};

float estimateDistance(int adc) {

    if (adc >= readings[0]) return 5.0;
    if (adc <= readings[6]) return 30.0;

    for (int i = 0; i < 6; i++) {

        if (adc <= readings[i] &&
            adc >= readings[i + 1]) {

            float ratio =
                (float)(readings[i] - adc) /
                (readings[i] - readings[i + 1]);

            return distances[i] +
                   ratio * (distances[i + 1] - distances[i]);
        }
    }

    return 30.0;
}

// ==================================================
// GYRO SETTINGS
// ==================================================

const float GYRO_SENSITIVITY = 131.0;

float gyroOffset = 0;
float gyroPolarity = 0;

float turnAngle = 0;

unsigned long lastGyroTime = 0;

float getTurnAngle() {
    return fabs(turnAngle);
}

// ==================================================
// GYRO 90 DEGREE TURN SETTINGS
// ==================================================

const int TURN_FAST = 120;
const int TURN_MEDIUM = 70;
const int TURN_SLOW = 50;

const int TURN_CORRECTION = 90;

const float TURN_TARGET = 90.0;

const float TURN_FAST_TARGET = 40.0;
const float TURN_MEDIUM_TARGET = 50.0;
const float TURN_BRAKE_ANGLE = 55.0;

const float TURN_TOLERANCE = 3.0;

const int MAX_CORRECTION_ATTEMPTS = 1;

const unsigned long FAST_TIMEOUT = 5000;
const unsigned long MEDIUM_TIMEOUT = 3000;
const unsigned long SLOW_TIMEOUT = 3000;

// ==================================================
// GYRO I2C
// ==================================================

bool readGyroZ(int16_t &result) {

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU_ADDR, 2) != 2) {
        return false;
    }

    uint16_t value = (uint16_t)Wire.read() << 8;
    value |= Wire.read();

    result = (int16_t)value;

    return true;
}

// ==================================================
// GYRO CALIBRATION
// ==================================================

bool calibrateGyro() {

    long total = 0;
    int validSamples = 0;

    Serial.println(F("Keep robot completely still..."));

    for (int i = 0; i < 200; i++) {

        int16_t raw;

        if (readGyroZ(raw)) {
            total += raw;
            validSamples++;
        }

        delay(5);
    }

    if (validSamples < 180) {

        Serial.println(F("ERROR: Gyro communication failed"));
        return false;
    }

    gyroOffset = (float)total / validSamples;

    Serial.print(F("Gyro offset: "));
    Serial.println(gyroOffset);

    return true;
}

// ==================================================
// GYRO ANGLE INTEGRATION
// ==================================================

void resetTurnMeasurement() {

    turnAngle = 0;
    gyroPolarity = 0;
    lastGyroTime = micros();
}

bool updateTurnAngle() {

    int16_t raw;

    if (!readGyroZ(raw)) {
        return false;
    }

    unsigned long now = micros();

    float dt = (now - lastGyroTime) / 1000000.0;

    lastGyroTime = now;

    if (dt <= 0 || dt > 0.1) {
        return true;
    }

    float rate =
        (raw - gyroOffset) / GYRO_SENSITIVITY;

    if (gyroPolarity == 0 && fabs(rate) > 2.0) {

        gyroPolarity = (rate > 0) ? 1.0 : -1.0;

        Serial.print(F("Gyro polarity: "));
        Serial.println(gyroPolarity);
    }

    if (gyroPolarity == 0) {
        return true;
    }

    turnAngle += rate * gyroPolarity * dt;

    return true;
}

// ==================================================
// MOTOR CONTROL
// ==================================================

void driveForward(int leftSpeed, int rightSpeed) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, constrain(leftSpeed, 0, 255));
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

void driveBackward(int speed) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void stopMotors() {

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

// Clockwise pivot
void pivotRight(int pwm) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);
}

// Counterclockwise pivot
void pivotLeft(int pwm) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);
}

// Active braking
void activeBrake() {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

// ==================================================
// FINISH LINE DETECTION - D5
// ==================================================

void checkFinishLine() {

    if (digitalRead(IR_FINISH) == BLACK_DETECTED) {

        Serial.println(F("BLACK TAPE DETECTED"));
        Serial.println(F("MAZE FINISH DETECTED"));

        driveForward(
            FINISH_FORWARD_SPEED,
            FINISH_FORWARD_SPEED
        );

        delay(FINISH_FORWARD_TIME);

        stopMotors();

        Serial.println(F("MAZE COMPLETED"));
        Serial.println(F("ROBOT STOPPED"));

        while (true) {
            stopMotors();
        }
    }
}

// ==================================================
// FRONT SENSOR
// ==================================================

bool frontBlocked() {

    VL53L0X_RangingMeasurementData_t measurement;

    tof.rangingTest(&measurement, false);

    if (measurement.RangeStatus == 4) {
        return false;
    }

    int distance = measurement.RangeMilliMeter;

    if (distance == 0) {
        return false;
    }

    Serial.print(F("Front distance: "));
    Serial.print(distance);
    Serial.println(F(" mm"));

    return distance < FRONT_THRESHOLD_MM;
}

// ==================================================
// LEFT WALL PID
// ==================================================

void followWall(float leftDistance) {

    float error = leftDistance - LEFT_TARGET;

    if (fabs(error) < DEADBAND) {
        error = 0;
    }

    int correction = constrain(
        (int)(KP * error),
        -MAX_CORRECTION,
        MAX_CORRECTION
    );

    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    driveForward(leftSpeed, rightSpeed);
}

// ==================================================
// REVERSE BEFORE TURN
// ==================================================

void moveBackwardBeforeTurn() {

    Serial.println(F("Reversing before turn"));

    driveBackward(REVERSE_SPEED);

    delay(REVERSE_TIME);

    stopMotors();

    delay(100);
}

// ==================================================
// BRAKE AND MEASURE
// ==================================================

bool brakeAndMeasure(unsigned long duration) {

    activeBrake();

    unsigned long start = millis();

    while (millis() - start < duration) {

        if (!updateTurnAngle()) {

            stopMotors();

            Serial.println(F("ERROR: Gyro read failed during braking"));

            return false;
        }

        delay(2);
    }

    stopMotors();

    return true;
}

// ==================================================
// GYRO TURN STAGE
// direction: 1 = RIGHT, -1 = LEFT
// ==================================================

bool runTurnStage(
    float targetAngle,
    int pwm,
    unsigned long timeout,
    int direction
) {

    unsigned long start = millis();
    unsigned long lastPrint = 0;

    while (getTurnAngle() < targetAngle) {

        if (millis() - start >= timeout) {

            stopMotors();

            Serial.println(F("TURN TIMEOUT"));

            Serial.print(F("Angle reached: "));
            Serial.println(getTurnAngle());

            return false;
        }

        if (direction == 1) {
            pivotRight(pwm);
        } else {
            pivotLeft(pwm);
        }

        if (!updateTurnAngle()) {

            stopMotors();

            Serial.println(F("GYRO ERROR - TURN ABORTED"));

            return false;
        }

        if (millis() - lastPrint >= 250) {

            Serial.print(F("Turn angle: "));
            Serial.println(getTurnAngle());

            lastPrint = millis();
        }

        delay(2);
    }

    stopMotors();

    return true;
}

// ==================================================
// GYRO CORRECTION PULSE
// ==================================================

bool correctionPulse(int direction) {

    unsigned long start = millis();

    if (direction == 1) {
        pivotRight(TURN_CORRECTION);
    } else {
        pivotLeft(TURN_CORRECTION);
    }

    while (millis() - start < 30) {

        if (!updateTurnAngle()) {

            stopMotors();

            return false;
        }

        delay(2);
    }

    stopMotors();

    return brakeAndMeasure(120);
}

// ==================================================
// GENERAL GYRO 90 DEGREE TURN
// direction: 1 = RIGHT, -1 = LEFT
// ==================================================

bool turn90(int direction) {

    if (direction == 1) {
        Serial.println(F("Starting 90 degree RIGHT turn"));
    } else {
        Serial.println(F("Starting 90 degree LEFT turn"));
    }

    resetTurnMeasurement();

    // FAST stage
    if (!runTurnStage(
            TURN_FAST_TARGET,
            TURN_FAST,
            FAST_TIMEOUT,
            direction)) {

        stopMotors();
        return false;
    }

    // MEDIUM stage
    if (!runTurnStage(
            TURN_MEDIUM_TARGET,
            TURN_MEDIUM,
            MEDIUM_TIMEOUT,
            direction)) {

        stopMotors();
        return false;
    }

    // SLOW stage
    if (!runTurnStage(
            TURN_BRAKE_ANGLE,
            TURN_SLOW,
            SLOW_TIMEOUT,
            direction)) {

        stopMotors();
        return false;
    }

    // Brake and measure
    if (!brakeAndMeasure(150)) {
        return false;
    }

    Serial.print(F("Angle after braking: "));
    Serial.println(getTurnAngle());

    // Correct if turn is short
    int attempts = 0;

    while (getTurnAngle() < TURN_TARGET - TURN_TOLERANCE &&
           attempts < MAX_CORRECTION_ATTEMPTS) {

        if (!correctionPulse(direction)) {
            return false;
        }

        attempts++;

        Serial.print(F("Turn correction angle: "));
        Serial.println(getTurnAngle());
    }

    // Correct if turn overshoots
    attempts = 0;

    while (getTurnAngle() > TURN_TARGET + TURN_TOLERANCE &&
           attempts < MAX_CORRECTION_ATTEMPTS) {

        if (!correctionPulse(-direction)) {
            return false;
        }

        attempts++;

        Serial.print(F("Reverse correction angle: "));
        Serial.println(getTurnAngle());
    }

    stopMotors();

    Serial.print(F("Final turn angle: "));
    Serial.println(getTurnAngle());

    Serial.print(F("Signed gyro angle: "));
    Serial.println(turnAngle);

    if (fabs(getTurnAngle() - TURN_TARGET) > TURN_TOLERANCE) {

        Serial.println(F("WARNING: Turn outside tolerance"));

        return false;
    }

    return true;
}

// ==================================================
// GYRO TURN HELPERS
// ==================================================

bool turnLeft90() {
    return turn90(-1);
}

bool turnRight90() {
    return turn90(1);
}

// ==================================================
// LEFT OPENING TURN
// NO GYRO - STEER UNTIL LEFT WALL IS FOUND
// NO TIMEOUT
// ==================================================

bool turnLeftUntilWallFound() {

    Serial.println(F("LEFT OPENING DETECTED"));
    Serial.println(F("Steering left until new wall is found"));

    leftOpeningArmed = false;
    leftOpenCount = 0;

    int wallCount = 0;

    while (true) {

        // Check finish line during the opening maneuver
        checkFinishLine();

        float leftDistance =
            estimateDistance(analogRead(IR_LEFT));

        // Adjustable left steering ratio
        driveForward(
            LEFT_TURN_LEFT_SPEED,
            LEFT_TURN_RIGHT_SPEED
        );

        if (leftDistance <= LEFT_WALL_FOUND_DISTANCE) {

            wallCount++;

            if (wallCount >= WALL_FOUND_COUNT) {

                stopMotors();

                Serial.println(F("NEW LEFT WALL FOUND"));
                Serial.println(F("Returning to normal PID"));

                return true;
            }

        } else {

            wallCount = 0;
        }

        delay(10);
    }
}

// ==================================================
// FRONT OBSTACLE RESPONSE
// REVERSE AND GYRO RIGHT 90 DEGREE TURN
// ==================================================

void handleFrontObstacle() {

    Serial.println(F("FRONT OBSTACLE DETECTED"));

    stopMotors();
    delay(200);

    leftOpeningArmed = false;
    leftOpenCount = 0;

    moveBackwardBeforeTurn();

    // Turn RIGHT when front is blocked
    bool success = turnRight90();

    stopMotors();

    frontCount = 0;

    if (!success) {
        Serial.println(F("WARNING: Right turn reported failure"));
    } else {
        Serial.println(F("90 DEGREE RIGHT TURN COMPLETED"));
    }

    Serial.println(F("Resuming left-wall following"));

    ignoreFrontUntil = millis() + 400;

    delay(100);
}

// ==================================================
// SETUP
// ==================================================

void setup() {

    Serial.begin(115200);

    Wire.begin();

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(IR_LEFT, INPUT);
    pinMode(IR_FINISH, INPUT);

    stopMotors();

    // Wake MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);

    if (Wire.endTransmission() != 0) {

        Serial.println(F("ERROR: Gyro not found"));

        while (1) {
            stopMotors();
            delay(100);
        }
    }

    delay(100);

    // Initialize VL53L0X
    Serial.println(F("Starting VL53L0X..."));

    if (!tof.begin()) {

        Serial.println(F("ERROR: VL53L0X not found"));

        while (1) {
            stopMotors();
            delay(100);
        }
    }

    Serial.println(F("VL53L0X ready"));

    // Calibrate gyro
    if (!calibrateGyro()) {

        Serial.println(F("GYRO CALIBRATION FAILED"));

        while (1) {
            stopMotors();
            delay(100);
        }
    }

    Serial.println(F("Maze robot ready"));

    delay(1000);
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop() {

    // ----------------------------------------------
    // 0. CHECK FINISH LINE FIRST
    // ----------------------------------------------

    checkFinishLine();

    // ----------------------------------------------
    // 1. FRONT OBSTACLE CHECK
    // ----------------------------------------------

    if (millis() >= ignoreFrontUntil && frontBlocked()) {
        frontCount++;
    } else {
        frontCount = 0;
    }

    // ----------------------------------------------
    // 2. FRONT BLOCKED: REVERSE AND GYRO RIGHT 90
    // ----------------------------------------------

    if (frontCount >= CONFIRM_COUNT) {

        handleFrontObstacle();

        return;
    }

    // ----------------------------------------------
    // 3. READ LEFT SHARP IR
    // ----------------------------------------------

    float leftDistance =
        estimateDistance(analogRead(IR_LEFT));

    // ----------------------------------------------
    // 4. REARM LEFT OPENING DETECTION
    // ----------------------------------------------

    if (!leftOpeningArmed &&
        leftDistance <= LEFT_REARM_DISTANCE) {

        leftOpeningArmed = true;

        Serial.println(F("Left wall detected - opening detection armed"));
    }

    // ----------------------------------------------
    // 5. LEFT OPENING DETECTION
    // ----------------------------------------------

    if (leftOpeningArmed &&
        leftDistance > LEFT_OPEN_THRESHOLD) {

        leftOpenCount++;

        // Continue straight while confirming opening
        driveForward(BASE_SPEED, BASE_SPEED);

        if (leftOpenCount >= LEFT_OPEN_CONFIRM_COUNT) {

            leftOpeningArmed = false;
            leftOpenCount = 0;

            turnLeftUntilWallFound();

            delay(100);

            return;
        }
    }

    // ----------------------------------------------
    // 6. NORMAL LEFT WALL FOLLOWING
    // ----------------------------------------------

    else {

        leftOpenCount = 0;

        followWall(leftDistance);
    }

    delay(10);
}
