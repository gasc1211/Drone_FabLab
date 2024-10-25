// Drone Code

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Setup Motors

#define  MOTOR1_PIN 7
#define  MOTOR2_PIN 6
#define  MOTOR3_PIN 5
#define  MOTOR4_PIN 4

#define MOTOR_POWER_INTERVAL 120
#define MIN_MOTOR_SPEED 1000
#define MAX_MOTOR_SPEED 2000
#define IDLE_MOTOR_SPEED 1180

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

// RF Receiver Setup
#define CE_PIN 8
#define CSN_PIN 9

RF24 Transmitter(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Setup Sensors
Adafruit_MPU6050 Sensors;

// Define mpu6050 variables
float RatePitch, RateRoll, RateYaw;
// float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
// float RateCalibrationNumber;

// Define motors throttle input
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float MotorSpeed1, MotorSpeed2, MotorSpeed3, MotorSpeed4;

void setup() {

    // Serial Comms initialization
    Serial.begin(9600);

    // Transmitter Setup
    Transmitter.begin();
    Transmitter.openReadingPipe(0, address);
    Transmitter.setPALevel(RF24_PA_MIN);
    Transmitter.startListening();

    // Sensors initialization
    Sensors.begin();
    Sensors.setAccelerometerRange(MPU6050_RANGE_4_G);   // Set accelerometer to +-2 G
    Sensors.setGyroRange(MPU6050_RANGE_500_DEG);        // Set gyroscope to +-500 deg/s
    Sensors.setFilterBandwidth(MPU6050_BAND_5_HZ);      // Set filter bandwidth to 5ghz

    // Motors Setup
    Motor1.attach(MOTOR1_PIN, 1000, 2000);
    Motor2.attach(MOTOR2_PIN, 1000, 2000);
    Motor3.attach(MOTOR3_PIN, 1000, 2000);
    Motor4.attach(MOTOR4_PIN, 1000, 2000);
    delay(500);

}

void loop() {

    // Update gyroscope and accelerometer values
    sensors_event_t a, g, temp;
    Sensors.getEvent(&a, &g, &temp);

    // Read Roll, Pitch and Yaw values from sensors and convert them into deg/s
    RateRoll = g.gyro.x * 57.2957;
    RatePitch = g.gyro.y * 57.2957;
    RateYaw = g.gyro.z * 57.2957;

    // TODO: Setup calibration for sensors  

    Serial.print("Roll: ");
    Serial.print(RateRoll);
    Serial.print(" Pitch: ");
    Serial.print(RatePitch);
    Serial.print(" Yaw: ");
    Serial.print(RateRoll);
    Serial.println();

    // Serial.print("Accelerometer Readings: ");
    // Serial.print("X: ");
    // Serial.print(a.acceleration.x);
    // Serial.print(" Y: ");
    // Serial.print(a.acceleration.y);
    // Serial.print(" Z: ");
    // Serial.print(a.acceleration.z);
    // Serial.println();

    // Serial.print("Termometer Reading: ");
    // Serial.print(temp.temperature);
    // Serial.println("°C");

    // Receive signal from controller
    // uint16_t receivedValues[] = { 0, 0, 0, 0, 0, 0 };
    // uint16_t signals[] = { 0, 0, 0, 0, 0, 0 };

    // Roll, Pitch and Yaw inputs
    float receivedValues[] = { 0, 0, 0, 0 };
    float inputs[] = { 0, 0, 0, 0 };

    if (Transmitter.available()) {
        Transmitter.read(&receivedValues, sizeof(receivedValues));

        // Check for idle state
        if (receivedValues == inputs) {
            for (int i = 0; i < sizeof(receivedValues) / sizeof(float); i++) {
                inputs[i] = IDLE_MOTOR_SPEED;
            }
        }
        // Update motor speeds based on inputs
        else {
            for (int i = 0; i < sizeof(receivedValues) / sizeof(float); i++) {
                // signals[i] = map(receivedValues[i], 512, 1024, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
                inputs[i] = map(fabs(receivedValues[i]), 0, 1, 0, MOTOR_POWER_INTERVAL);

                if (receivedValues[i] < 0) {
                    inputs[i] *= -1;
                }
            }
        }
    }

    // TODO: Setup Control Loop and Input Error Correction
    // MotorInput1 = 0;
    // MotorInput2 = 0;
    // MotorInput3 = 0;
    // MotorInput4 = 0;

    Serial.print("Received values: ");
    Serial.print(" Roll: ");
    Serial.print(inputs[0]);
    Serial.print(" Pitch: ");
    Serial.print(inputs[1]);
    Serial.print(" Yaw: ");
    Serial.print(inputs[2]);
    Serial.println();

    // // Send speed signal to motors
    // Motor1.writeMicroseconds(inputs[3]);
    // Motor2.writeMicroseconds(inputs[3]);
    // Motor3.writeMicroseconds(inputs[3]);
    // Motor4.writeMicroseconds(inputs[3]);

    // Setup a 250Hz Closed Control Loop
    delay(400);
}
