// Drone Code
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include<Utils.h>

// Setup Motors (Labeled counterclockwise from the lower right corner of the drone)
#define  MOTOR1_PIN 3   // Motor 1
#define  MOTOR2_PIN 4   // Motor 2
#define  MOTOR3_PIN 5   // Motor 3
#define  MOTOR4_PIN 6   // Motor 4

#define MOTOR_POWER_INTERVAL 10
#define MIN_MOTOR_SPEED 1000
#define MAX_MOTOR_SPEED 1300
#define IDLE_MOTOR_SPEED 1050

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

int16_t ThrottleInput;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// RF Receiver Setup
#define CE_PIN 7
#define CSN_PIN 8

RF24 Transmitter(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Setup Sensors
#define ROTAION_INTERVAL 75
Adafruit_MPU6050 Sensors;

// Differential Term Variables
float ErrorRatePitch, ErrorRateRoll, ErrorRateYaw;
float RatePitch, RateRoll, RateYaw;
float DesiredRatePitch, DesiredRateRoll, DesiredRateYaw;

// Calibration variables
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float RateCalibrationNumber;

// Define motor individual movement input
float PitchInput1, PitchInput2, PitchInput3, PitchInput4;
float RollInput1, RollInput2, RollInput3, RollInput4;
float YawInput1, YawInput2, YawInput3, YawInput4;

void gyro_signals() {
    // Update gyroscope and accelerometer values
    sensors_event_t a, g, temp;
    Sensors.getEvent(&a, &g, &temp);

    // Read Roll, Pitch and Yaw values from sensors and convert them into deg/s
    RateRoll = g.gyro.x * 57.2958;
    RatePitch = g.gyro.y * 57.2958;
    RateYaw = g.gyro.z * 57.2958;

    // Log readings to serial monitor
    // Serial.print("Roll: ");
    // Serial.print(RateRoll);
    // Serial.print(" Pitch: ");
    // Serial.print(RatePitch);
    // Serial.print(" Yaw: ");
    // Serial.print(RateYaw);
    // Serial.println();
}

void setup() {

    // Serial Comms initialization
    Serial.begin(9600);

    // Transmitter Setup
    Transmitter.begin();
    Transmitter.openReadingPipe(0, address);
    Transmitter.setPALevel(RF24_PA_MIN);
    Transmitter.startListening();

    // Gyroscope initialization
    Sensors.begin();
    Sensors.setAccelerometerRange(MPU6050_RANGE_4_G);   // Set accelerometer to +-2 G
    Sensors.setGyroRange(MPU6050_RANGE_500_DEG);        // Set gyroscope to +-500 deg/s
    Sensors.setFilterBandwidth(MPU6050_BAND_5_HZ);      // Set filter bandwidth to 5ghz

    // Gyroscope Calibration
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;

    // Motors Setup
    Motor1.attach(MOTOR1_PIN, 1000, 2000);
    Motor2.attach(MOTOR2_PIN, 1000, 2000);
    Motor3.attach(MOTOR3_PIN, 1000, 2000);
    Motor4.attach(MOTOR4_PIN, 1000, 2000);
    delay(100);
}

void loop() {
    // Get Gyroscope measurements
    gyro_signals();

    // Correct for deviation using calibration values
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    // Log readings to serial monitor
    // Serial.print("X: ");
    // Serial.print(a.acceleration.x);
    // Serial.print(" Y: ");
    // Serial.print(a.acceleration.y);
    // Serial.print(" Z: ");
    // Serial.print(a.acceleration.z);
    // Serial.println();

    // Roll, Pitch, Yaw and Throttle inputs
    int16_t receivedValues[] = { 0, 0, 0, 0 };
    int16_t inputs[] = { 0, 0, 0, 0 };

    if (Transmitter.available()) {
        Transmitter.read(&receivedValues, sizeof(receivedValues));

        // Check for idle state
        // if (receivedValues == inputs) {
        //     for (int i = 0; i < sizeof(receivedValues) / sizeof(int16_t); i++) {
        //         inputs[i] = IDLE_MOTOR_SPEED;
        //     }
        // }
        // Update motor speeds based on inputs
        // else {
        //     for (int i = 0; i < sizeof(receivedValues) / sizeof(int16_t); i++) {
        //         inputs[i] = map(receivedValues[i], 0, 100, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
        //     }
        // }

        for (int i = 0; i < sizeof(receivedValues) / sizeof(int16_t); i++) {
            // inputs[i] = map(receivedValues[i], 0, 100, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
            inputs[i] = receivedValues[i];
        }

        // TODO: Setup Control Loop and Input Error Correction

        DesiredRatePitch = mapf(inputs[0], 0, 100, -ROTAION_INTERVAL, ROTAION_INTERVAL);
        DesiredRateRoll = mapf(inputs[1], 0, 100, -ROTAION_INTERVAL, ROTAION_INTERVAL);
        DesiredRateYaw = mapf(inputs[2], 0, 100, -ROTAION_INTERVAL, ROTAION_INTERVAL);
        ThrottleInput = map(inputs[3], 0, 100, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

        // Temporal direct override of motor input using only throttle
        MotorInput1 = ThrottleInput + 50;
        MotorInput2 = ThrottleInput;
        MotorInput3 = ThrottleInput;
        MotorInput4 = ThrottleInput;

        // Log Received Values
        Serial.print("Received values: ");
        // Serial.print(" Pitch: ");
        // Serial.print(DesiredRatePitch);
        // Serial.print(" Roll: ");
        // Serial.print(DesiredRateRoll);
        // Serial.print(" Yaw: ");
        // Serial.print(DesiredRateYaw);
        Serial.print(" Input[3]: ");
        Serial.print(inputs[3]);
        Serial.print(" Throttle: ");
        Serial.print(ThrottleInput);
        Serial.println();

    }

    // Send speed signal to motors
    Motor1.writeMicroseconds(MotorInput1);
    Motor2.writeMicroseconds(MotorInput2);
    Motor3.writeMicroseconds(MotorInput3);
    Motor4.writeMicroseconds(MotorInput4);

    // Setup a 250Hz Closed Control Loop
    delay(4);
}
