
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

// Right Sharp IR
#define IR_RIGHT A0

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
// RIGHT WALL PID SETTINGS
// ==================================================

const float RIGHT_TARGET = 8.0;
const float DEADBAND = 2.0;

const int BASE_SPEED = 170;
const int MAX_CORRECTION = 120;

const float KP = 5.0;

int previousCorrection = 0;

// ==================================================
// RIGHT OPENING DETECTION
// ==================================================

const float RIGHT_OPEN_THRESHOLD = 15.0;
const float RIGHT_REARM_DISTANCE = 11.0;

const int RIGHT_OPEN_CONFIRM_COUNT = 2;

int rightOpenCount = 0;
bool rightOpeningArmed = true;

// ==================================================
// RIGHT TURN SETTINGS
// ==================================================

const int RIGHT_TURN_LEFT_SPEED = 150;
const int RIGHT_TURN_RIGHT_SPEED = 50;

const float RIGHT_WALL_FOUND_DISTANCE = 10.0;

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
// LEFT 90 DEGREE TURN SETTINGS
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

        // Move forward slightly
        driveForward(
            FINISH_FORWARD_SPEED,
            FINISH_FORWARD_SPEED
        );

        delay(FINISH_FORWARD_TIME);

        // Stop motors
        stopMotors();

        Serial.println(F("MAZE COMPLETED"));
        Serial.println(F("ROBOT STOPPED"));

        // Permanently stop the program
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
// RIGHT WALL PID
// ==================================================

void followWall(float rightDistance) {

    float error = rightDistance - RIGHT_TARGET;

    if (fabs(error) < DEADBAND) {
        error = 0;
    }

    int correction = constrain(
        (int)(KP * error),
        -MAX_CORRECTION,
        MAX_CORRECTION
    );

    int leftSpeed = BASE_SPEED + correction;
    int rightSpeed = BASE_SPEED - correction;

    driveForward(leftSpeed, rightSpeed);
}

// ==================================================
// REVERSE BEFORE LEFT TURN
// ==================================================

void moveBackwardBeforeLeftTurn() {

    Serial.println(F("Reversing before LEFT turn"));

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

    // Brake and measure actual angle
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
// LEFT 90 DEGREE TURN
// ==================================================

bool turnLeft90() {
    return turn90(-1);
}

// ==================================================
// RIGHT OPENING TURN
// NO GYRO - STEER UNTIL RIGHT WALL IS FOUND
// ==================================================

bool turnRightUntilWallFound() {

    Serial.println(F("RIGHT OPENING DETECTED"));
    Serial.println(F("Steering right until new wall is found"));

    rightOpeningArmed = false;
    rightOpenCount = 0;

    int wallCount = 0;

    while (true) {

        float rightDistance =
            estimateDistance(analogRead(IR_RIGHT));

        // Adjustable right steering ratio
        driveForward(
            RIGHT_TURN_LEFT_SPEED,
            RIGHT_TURN_RIGHT_SPEED
        );

        if (rightDistance <= RIGHT_WALL_FOUND_DISTANCE) {

            wallCount++;

            if (wallCount >= WALL_FOUND_COUNT) {

                stopMotors();

                Serial.println(F("NEW RIGHT WALL FOUND"));
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
// USES ORIGINAL MPU6050 90 DEGREE LEFT TURN
// ==================================================

void handleFrontObstacle() {

    Serial.println(F("FRONT OBSTACLE DETECTED"));

    stopMotors();
    delay(200);

    rightOpeningArmed = false;
    rightOpenCount = 0;

    moveBackwardBeforeLeftTurn();

    bool success = turnLeft90();

    stopMotors();

    frontCount = 0;

    if (!success) {
        Serial.println(F("WARNING: Left turn reported failure"));
    } else {
        Serial.println(F("90 DEGREE LEFT TURN COMPLETED"));
    }

    Serial.println(F("Resuming right-wall following"));

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

    pinMode(IR_RIGHT, INPUT);
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
    // 2. FRONT BLOCKED: REVERSE AND GYRO LEFT 90
    // ----------------------------------------------

    if (frontCount >= CONFIRM_COUNT) {

        handleFrontObstacle();

        return;
    }

    // ----------------------------------------------
    // 3. READ RIGHT SHARP IR
    // ----------------------------------------------

    float rightDistance =
        estimateDistance(analogRead(IR_RIGHT));

    // ----------------------------------------------
    // 4. REARM RIGHT OPENING DETECTION
    // ----------------------------------------------

    if (!rightOpeningArmed &&
        rightDistance <= RIGHT_REARM_DISTANCE) {

        rightOpeningArmed = true;

        Serial.println(F("Right wall detected - opening detection armed"));
    }

    // ----------------------------------------------
    // 5. RIGHT OPENING DETECTION
    // ----------------------------------------------

    if (rightOpeningArmed &&
        rightDistance > RIGHT_OPEN_THRESHOLD) {

        rightOpenCount++;

        // Continue straight while confirming opening.
        driveForward(BASE_SPEED, BASE_SPEED);

        if (rightOpenCount >= RIGHT_OPEN_CONFIRM_COUNT) {

            rightOpeningArmed = false;
            rightOpenCount = 0;

            turnRightUntilWallFound();

            delay(100);

            return;
        }
    }

    // ----------------------------------------------
    // 6. NORMAL RIGHT WALL FOLLOWING
    // ----------------------------------------------

    else {

        rightOpenCount = 0;

        followWall(rightDistance);
    }

    delay(10);
}
