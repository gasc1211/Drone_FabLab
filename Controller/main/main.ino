
// Controller Code
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Setup Joysticks
#define STICKL_X A0
#define STICKL_Y A1
#define SWL 3

#define STICKR_X A2
#define STICKR_Y A3
#define SWR 2

int StickLXMidPoint, StickRXMidPoint, StickLYMidPoint, StickRYMidPoint;

float PitchInput, RollInput, YawInput, ThrottleInput;

// Setup transmitter
#define CE_PIN 7
#define CSN_PIN 8

RF24 Transmitter(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

void setup() {

  // Initialize serial comms
  Serial.begin(9600);
  Serial.println("Starting controller...");

  // Initialize transmitter
  Serial.println("  [!] Initializing transmitter...");
  Transmitter.begin();
  Transmitter.openWritingPipe(address);
  Transmitter.setPALevel(RF24_PA_MIN);
  Transmitter.stopListening();
  Serial.println("   ...transmitter initialization done!");

  // Calibrate Sticks midpoint
  Serial.println("  [!] Calibrating joysticks center point...");
  StickLXMidPoint = analogRead(STICKL_X);
  StickRXMidPoint = analogRead(STICKR_X);
  StickLYMidPoint = analogRead(STICKL_Y);
  StickRYMidPoint = analogRead(STICKR_Y);
  Serial.println("   ...Joysticks calibration done!");

  Serial.println("...Controller setup done!");
}

void loop() {
  // Read input from joysticks
  uint16_t stickL_X = analogRead(STICKL_X);
  uint16_t stickL_Y = analogRead(STICKL_Y);
  uint16_t stickR_X = analogRead(STICKR_X);
  uint16_t stickR_Y = analogRead(STICKR_Y);
  int swL = digitalRead(SWL);
  int swR = digitalRead(SWR);

  // Test Joysticks
  // Serial.print("LX: ");
  // Serial.print(stickL_X);
  // Serial.print(" RY: ");
  // Serial.print(stickL_Y);
  // Serial.print(" RX: ");
  // Serial.print(stickR_X);
  // Serial.print(" RY: ");
  // Serial.print(stickR_Y);
  // Serial.println();

  // RollInput
  if (stickR_X >= StickRXMidPoint) {
    RollInput = map(stickR_X, StickRXMidPoint, 1024, 0, 100) / 100;
  }
  else if (stickR_X < StickRXMidPoint) {
    RollInput = -map(stickR_X, StickRXMidPoint, 0, 0, 100) / 100;
  }

  // PitchInput
  if (stickR_Y >= StickRYMidPoint) {
    PitchInput = map(stickR_Y, StickRYMidPoint, 1024, 0, 100) / 100;
  }
  else if (stickR_Y < 512) {
    PitchInput = -map(stickR_Y, 512, 0, 0, 100) / 100;
  }

  // YawInput
  if (stickL_X >= StickLXMidPoint) {
    YawInput = map(stickL_X, StickLXMidPoint, 1024, 0, 100) / 100;
  }
  else if (stickL_X < StickLXMidPoint) {
    YawInput = -map(stickL_X, StickLXMidPoint, 0, 0, 100) / 100;
  }

  // ThrottleInput
  if (stickL_Y >= StickLYMidPoint) {
    YawInput = map(stickL_Y, StickLYMidPoint, 1024, 0, 100) / 100;
  }
  else if (stickL_Y < StickLYMidPoint) {
    YawInput = -map(stickL_Y, StickLYMidPoint, 0, 0, 100) / 100;
  }

  // Send inputs through nrf24L01
  float commands[] = { PitchInput, RollInput, YawInput, ThrottleInput };
  Transmitter.write(&commands, sizeof(commands));


  // uint16_t inputs[] = { stickL_X, stickL_Y, stickR_X, stickR_Y, (uint16_t)swL, (uint16_t)swR };
  // transmitter.write(&inputs, sizeof(inputs));

  // Send signals in 250Hz 
  delay(400);
}
