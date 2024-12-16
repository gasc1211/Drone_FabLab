// Controller Code
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Setup Joysticks
#define STICKL_Y A0
#define STICKL_X A1
#define SWL 3

#define STICKR_Y A2
#define STICKR_X A3
#define SWR 2

int StickLXMidPoint, StickRXMidPoint, StickLYMidPoint, StickRYMidPoint;
int16_t PitchInput, RollInput, YawInput, ThrottleInput;

// Setup transmitter
#define CE_PIN 7
#define CSN_PIN 8

RF24 Transmitter(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

void setup() {

  // Initialize serial comms
  Serial.begin(9600);
  Serial.println("Starting controller...");

  // Setup Joystick buttons
  Serial.println("  [!] Initializing joystick buttons...");
  pinMode(SWL, INPUT_PULLUP);
  pinMode(SWR, INPUT_PULLUP);
  Serial.println("   ...joystick buttons initialization done!");

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
  Serial.print("LX: ");
  Serial.print(stickL_X);
  Serial.print(" LY: ");
  Serial.print(stickL_Y);
  Serial.print(" RX: ");
  Serial.print(stickR_X);
  Serial.print(" RY: ");
  Serial.print(stickR_Y);
  Serial.print(" L3: ");
  // Serial.print(digitalRead(SWL));
  Serial.print(swL);
  Serial.print(" R3: ");
  // Serial.print(digitalRead(SWR));
  Serial.print(swR);
  Serial.println();

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
    ThrottleInput = map(stickL_Y, StickLYMidPoint, 1024, 0, 100);
  }
  else if (stickL_Y < StickLYMidPoint) {
    ThrottleInput = -map(stickL_Y, StickLYMidPoint, 0, 0, 100);
  }

  // Send inputs through nrf24L01
  Serial.println(ThrottleInput);
  int16_t commands[] = { RollInput, PitchInput, YawInput, ThrottleInput };
  Transmitter.write(&commands, sizeof(commands));

  // Send signals in 250Hz 
  delay(4);
}
