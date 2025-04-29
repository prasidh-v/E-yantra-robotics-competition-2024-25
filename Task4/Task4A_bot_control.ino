#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SoftwareSerial.h>

/* ====================== */
/* ==== VARIABLES ======= */
/* ====================== */

// Bluetooth Communication
SoftwareSerial bluetooth(A1, A0);  // RX, TX (HC-05 TX->A0, RX->A1)

// Motor Driver Pins
#define ENA 6    // Left motor PWM pin (Range: 0-255)
#define IN1 A2   // Left motor direction pin 1
#define IN2 A3   // Left motor direction pin 2
#define ENB 5    // Right motor PWM pin (Range: 0-255)
#define IN3 9    // Right motor direction pin 1
#define IN4 4    // Right motor direction pin 2

// Encoder Pins
#define ENCODER1_CHANNEL_A 2  // Left encoder channel A (Interrupt pin)
#define ENCODER1_CHANNEL_B 7  // Left encoder channel B
#define ENCODER2_CHANNEL_A 3  // Right encoder channel A (Interrupt pin)
#define ENCODER2_CHANNEL_B 8  // Right encoder channel B

// PID Constants
#define Kp_angle  225   // Proportional gain for angle control
#define Kd_angle  9     // Derivative gain for angle control
#define Ki_angle  7     // Integral gain for angle control
#define Kp_speed  49.0  // Proportional gain for speed control
#define Ki_speed  0.26  // Integral gain for speed control
#define Kp_position 44  // Proportional gain for position control
#define Ki_position 0.0 // Integral gain for position control
#define Kd_position 10.0// Derivative gain for position control

// Control Parameters
#define sampleTime  0.010  // Control loop period in seconds
#define targetAngle 4.0    // Target balancing angle in degrees
#define TURN_GAIN 40       // Turning sensitivity multiplier

// Bluetooth Command Definitions
#define FORWARD 'F'    // Increase target speed
#define BACKWARD 'B'   // Decrease target speed
#define LEFT 'L'       // Left turn command
#define RIGHT 'R'      // Right turn command
#define CIRCLE 'C'     // Clockwise spin command
#define CROSS 'X'      // Emergency stop
#define TRIANGLE 'T'   // Reset position
#define SQUARE 'S'     // Gradual stop
#define START 'A'      // Enable movement
#define PAUSE 'P'      // Disable movement

// MPU6050 Object
MPU6050 mpu;

// IMU Data Variables
bool dmpReady = false;         // DMP initialization status
uint8_t mpuIntStatus;          // MPU interrupt status
uint8_t devStatus;             // Device status
uint16_t packetSize;           // DMP packet size
uint8_t fifoBuffer[64];        // FIFO storage buffer
Quaternion q;                  // Quaternion container
VectorFloat gravity;           // Gravity vector
float ypr[3];                  // Yaw/Pitch/Roll array
const float conv = 180.0/M_PI; // Radians to degrees

// Motor Control Variables
volatile int leftMotorPower;   // Left motor power (-255 to 255)
volatile int rightMotorPower;  // Right motor power (-255 to 255)

// Angle Control Variables
volatile float currentAngle = 0; // Filtered tilt angle (degrees)
volatile float prevAngle = 0;    // Previous raw angle
volatile float error = 0;        // Current angle error
volatile float prevError = 0;    // Previous angle error
volatile float errorSum = 0;     // Cumulative angle error

// Speed Control Variables
volatile long leftEncoderCount = 0;  // Left encoder ticks
volatile long rightEncoderCount = 0; // Right encoder ticks
volatile long prevLeftCount = 0;     // Previous left encoder count
volatile long prevRightCount = 0;    // Previous right encoder count
volatile float speed_filter = 0;     // Filtered speed value
volatile float speed_filter_old = 0; // Previous filtered speed
volatile float car_speed_integeral = 0; // Speed error integral
volatile int speed_control_period_count = 0; // Speed control counter
volatile float speed_control_output = 0; // Speed PID output
volatile int setting_car_speed = 0;     // Target speed (-150 to 150)

// Position Control Variables
volatile long initialPosition = 0;    // Reference position
volatile long currentPosition = 0;    // Current position
volatile float positionError = 0;     // Position error
volatile float positionErrorSum = 0;  // Cumulative position error
volatile float positionDerivative = 0;// Position error derivative
volatile float prevPositionError = 0; // Previous position error

// Navigation Control
volatile int turnOffset = 0;  // Turning adjustment (-TURN_GAIN to TURN_GAIN)

/* ====================== */
/* ==== FUNCTIONS ======= */
/* ====================== */

/**
 * Function Name: init_PID
 * Input: None
 * Output: None
 * Logic: Initializes Timer1 for 100Hz interrupt to run PID control
 * Example Call: init_PID();
 */
void init_PID() {
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 9999;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

/**
 * Function Name: leftEncoderISR
 * Input: None (Interrupt Service Routine)
 * Output: None
 * Logic: Updates left encoder count based on channel B state
 * Example Call: Automatically called on encoder pulse
 */
void leftEncoderISR() { 
    if (digitalRead(ENCODER1_CHANNEL_B)) {
        leftEncoderCount++;
    } else {
        leftEncoderCount--;
    }
}

/**
 * Function Name: rightEncoderISR
 * Input: None (Interrupt Service Routine)
 * Output: None
 * Logic: Updates right encoder count based on channel B state
 * Example Call: Automatically called on encoder pulse
 */
void rightEncoderISR() {
    if (digitalRead(ENCODER2_CHANNEL_B)) {
        rightEncoderCount--;
    } else {
        rightEncoderCount++;
    }
}

/**
 * Function Name: setMotors
 * Input: leftMotorSpeed (-255 to 255), rightMotorSpeed (-255 to 255)
 * Output: None
 * Logic: Sets motor directions and speeds using PWM
 * Example Call: setMotors(150, -150);
 */
void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    // Left motor control
    if (leftMotorSpeed >= 0) {
        analogWrite(ENA, leftMotorSpeed);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    } else {
        analogWrite(ENA, -leftMotorSpeed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }

    // Right motor control
    if (rightMotorSpeed >= 0) {
        analogWrite(ENB, rightMotorSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        analogWrite(ENB, -rightMotorSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
}

/**
 * Function Name: updatePosition
 * Input: None
 * Output: None
 * Logic: Calculates current position and updates PID terms
 * Example Call: updatePosition();
 */
void updatePosition() {
    currentPosition = (leftEncoderCount + rightEncoderCount) / 2;
    positionError = initialPosition - currentPosition;
    positionDerivative = (positionError - prevPositionError) / sampleTime;
    positionErrorSum += positionError * sampleTime;
    positionErrorSum = constrain(positionErrorSum, -100, 100);
    prevPositionError = positionError;
}

/**
 * Function Name: updateMPUData
 * Input: None
 * Output: bool - True if new data processed
 * Logic: Reads IMU data from FIFO and calculates tilt angle
 * Example Call: if(updateMPUData()) { /* new data available */
bool updateMPUData() {
    uint16_t fifoCount = mpu.getFIFOCount();
    
    if (fifoCount >= 1024) {
        mpu.resetFIFO();
        return false;
    }
    
    if (fifoCount >= packetSize) {
        while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        float newAngle = ypr[1] * conv;
        currentAngle = 0.93 * (currentAngle + newAngle - prevAngle) + 0.07 * newAngle;
        prevAngle = newAngle;
        
        return true;
    }
    return false;
}

/**
 * Function Name: processBluetoothCommand
 * Input: char command - Received command byte
 * Output: None
 * Logic: Maps Bluetooth commands to control actions
 * Example Call: processBluetoothCommand('F');
 */
void processBluetoothCommand(char command) {
    switch(command) {
        case FORWARD:  // Increase target speed
            setting_car_speed += 30;
            break;
        case BACKWARD:  // Decrease target speed
            setting_car_speed -= 30;
            break;
        case LEFT:  // Left turn adjustment
            turnOffset = TURN_GAIN;
            break;
        case RIGHT:  // Right turn adjustment
            turnOffset = -TURN_GAIN;
            break;
        case CIRCLE:  // Clockwise spin
            turnOffset = -2 * TURN_GAIN;
            setting_car_speed = 0;
            break;
        case CROSS:  // Emergency stop
            setting_car_speed = 0;
            turnOffset = 0;
            leftMotorPower = rightMotorPower = 0;
            break;
        case TRIANGLE:  // Reset position reference
            initialPosition = (leftEncoderCount + rightEncoderCount) / 2;
            break;
        case SQUARE:  // Gradual stop
            setting_car_speed = 0;
            turnOffset = 0;
            break;
        case START:  // Enable movement
            setting_car_speed = 20;
            break;
        case PAUSE:  // Disable movement
            setting_car_speed = 0;
            turnOffset = 0;
            break;
    }
    setting_car_speed = constrain(setting_car_speed, -150, 150);
}

/* ====================== */
/* ==== MAIN CODE ======= */
/* ====================== */

void setup() {
    Wire.begin();
    Wire.setClock(400000);

    // Motor control pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Encoder pins
    pinMode(ENCODER1_CHANNEL_A, INPUT_PULLUP);
    pinMode(ENCODER1_CHANNEL_B, INPUT_PULLUP);
    pinMode(ENCODER2_CHANNEL_A, INPUT_PULLUP);
    pinMode(ENCODER2_CHANNEL_B, INPUT_PULLUP);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER1_CHANNEL_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_CHANNEL_A), rightEncoderISR, RISING);

    Serial.begin(9600);
    bluetooth.begin(9600);

    // MPU6050 initialization
    mpu.initialize();
    if (!mpu.testConnection()) {
        while (1);
    }

    // IMU configuration
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    
    // Calibration offsets
    mpu.setXAccelOffset(-2495);
    mpu.setYAccelOffset(1215);
    mpu.setZAccelOffset(1720);
    mpu.setXGyroOffset(110);
    mpu.setYGyroOffset(47);
    mpu.setZGyroOffset(-32);

    // DMP initialization
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        while (1);
    }

    init_PID();
    
    // Position control initialization
    initialPosition = (leftEncoderCount + rightEncoderCount) / 2;
    currentPosition = initialPosition;
}

void loop() {
    if (!dmpReady) return;

    // Handle Bluetooth input
    if (bluetooth.available()) {
        char command = bluetooth.read();
        processBluetoothCommand(command);
    }

    // Update IMU data
    updateMPUData();

    // Apply motor controls
    leftMotorPower = constrain(leftMotorPower, -255, 255);
    rightMotorPower = constrain(rightMotorPower, -255, 255);
    setMotors(leftMotorPower, rightMotorPower);
}

/**
 * Function Name: ISR(TIMER1_COMPA_vect)
 * Input: None (Timer1 interrupt)
 * Output: None
 * Logic: Main control loop running at 100Hz
 *        1. Calculates angle PID
 *        2. Calculates speed PID
 *        3. Combines controls with turning offset
 * Example Call: Automatically called by timer interrupt
 */
ISR(TIMER1_COMPA_vect) {
    // Angle PID calculation
    error = targetAngle - currentAngle;
    float derivative = (error - prevError) / sampleTime;
    errorSum += error;
    errorSum = constrain(errorSum, -300, 300);
    float integral = errorSum * sampleTime;
    
    int basePower = Kp_angle * error + Ki_angle * integral + Kd_angle * derivative;
    
    // Speed control (runs at 12.5Hz)
    speed_control_period_count++;
    if (speed_control_period_count >= 8) {
        speed_control_period_count = 0;
        float car_speed = (leftEncoderCount - prevLeftCount + rightEncoderCount - prevRightCount) * 0.5;
        prevLeftCount = leftEncoderCount;
        prevRightCount = rightEncoderCount;
        
        // Low-pass filter for speed
        speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
        speed_filter_old = speed_filter;
        
        // Integral term with anti-windup
        car_speed_integeral += speed_filter;
        car_speed_integeral += -setting_car_speed;
        car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
        
        // Speed PID calculation
        speed_control_output = -Kp_speed * speed_filter - Ki_speed * car_speed_integeral;
    }
    
    // Motor power calculation with turn offset
    leftMotorPower = basePower - speed_control_output - turnOffset;
    rightMotorPower = basePower - speed_control_output + turnOffset;
    
    prevError = error;
}