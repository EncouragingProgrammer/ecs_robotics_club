/**********************************************************************
  Filename    : Multifunctional_RF24_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : A Multifunctional RF24-Remote Car with Servo controlled by POT1.
  Author      : www.freenove.com
  Modification: 2019/08/08 (Original), 2025/XX/XX (Cleanup & Servo control)
  
  Explanation:
    - This sketch controls a Freenove 4WD car with:
       1) Motor driving (forward/backward + turning).
       2) Servo steering control (controlled by POT1 on the remote).
       3) Speed scaling (speed_reduction), to limit maximum speed.
       4) Buzzer activation (joystick Z button on remote).
       5) LED strip initialization (all red).

    - The car listens for signals via an nRF24L01 wireless module.
    - The remote must use the same "addresses" as the car, otherwise
      they will not communicate.

**********************************************************************/

#include "Freenove_WS2812B_RGBLED_Controller.h"
#include "RF24.h"
#include <Servo.h>
#include <FlexiTimer2.h>

// --------------------- Pin Definitions ---------------------
/*
  These are the Arduino pins connected to specific parts of the car:
   - PIN_SERVO:        Servo signal pin
   - PIN_DIRECTION_*:  Direction pins for left/right motors
   - PIN_MOTOR_PWM_*:  PWM (speed) pins for left/right motors
   - PIN_BUZZER:       Buzzer pin (shared with battery input in original design)
   - PIN_TRACKING_*:   (Optional) Line tracking sensors, if used
*/
#define PIN_SERVO             2
#define PIN_DIRECTION_LEFT    4
#define PIN_DIRECTION_RIGHT   3
#define PIN_MOTOR_PWM_LEFT    6
#define PIN_MOTOR_PWM_RIGHT   5

#define MOTOR_DIRECTION       0  // Set to 1 if motors spin in the wrong direction
#define MOTOR_PWM_DEAD        10 // Small PWM "dead zone" so the motors don't hum

// nRF24L01 pins
#define PIN_SPI_CE            9
#define PIN_SPI_CSN           10
// On Arduino UNO, pins 11,12,13 are reserved for SPI (MOSI, MISO, SCK)

// Buzzer pin (shared with battery pin in original code)
#define PIN_BUZZER            A0  

// Optional line-tracking sensors
#define PIN_TRACKING_LEFT     A1
#define PIN_TRACKING_CENTER   A2
#define PIN_TRACKING_RIGHT    A3

// --------------------- LED Strip Definitions ----------------
/*
  We have a built-in Freenove WS2812B controller on I2C address 0x20,
  controlling STRIP_LEDS_COUNT LEDs. In this example, we simply set them
  all to red in setup().
*/
#define STRIP_I2C_ADDRESS     0x20
#define STRIP_LEDS_COUNT      10
Freenove_WS2812B_Controller strip(STRIP_I2C_ADDRESS, STRIP_LEDS_COUNT, TYPE_GRB);

// --------------------- nRF24L01 Globals ---------------------
/*
  The RF24 library is used for wireless communications.
  "addresses" must match on both the car and the remote to communicate.
  If you have multiple cars, you must give each pair a unique address
  (e.g. "Car01", "Car02", "BotA", "BotB", etc.) so they don't conflict.
*/
RF24 radio(PIN_SPI_CE, PIN_SPI_CSN);
const byte addresses[6] = "Laser";  // 5-character address + terminator
// e.g. "Car01", "BotA!" , etc.

// Data layout from remote: nrfDataRead[8]
/*
  The remote typically sends an array of 8 integers:
   0: POT1 (0..1023)
   1: POT2 (0..1023)
   2: JOYSTICK_X (0..1023)
   3: JOYSTICK_Y (0..1023)
   4: JOYSTICK_Z (0 or 1)
   5: S1 (switch)
   6: S2 (switch)
   7: S3 (switch)
*/
enum RemoteData {
  POT1        = 0,   // 0..1023
  POT2        = 1,
  JOYSTICK_X  = 2,
  JOYSTICK_Y  = 3,
  JOYSTICK_Z  = 4,
  S1          = 5,
  S2          = 6,
  S3          = 7
};
int nrfDataRead[8];
bool nrfComplete = false; // Tells us if new data just arrived

#define NRF_UPDATE_TIMEOUT 1000
unsigned long lastNrfUpdateTime = 0;

// --------------------- Servo & Speed Reduction -------------------------------
/*
  The servo is controlled by the value of POT1 on the remote.
  We also have a speed_reduction value (0.6..1.0) that scales the motor speeds.
  - 1.0 = full speed
  - 0.6 = 60% speed (might be good for "jousting" to reduce collisions).
*/
Servo servo;
int servoOffset = 0;         // Adjust if servo center is off
float speed_reduction = 0.8; // Adjust this for top speed (0.6 to 1.0, for example)

// --------------------- Function Prototypes ------------------
void pinsSetup();
void servoSetup();
bool nrf24L01Setup();
void checkNrfReceived();
bool getNrf24L01Data();
void clearNrfFlag();
void updateCarActionByNrfRemote();
void resetNrfDataBuf();
void motorRun(int speedl, int speedr);
void setBuzzer(bool flag);
void alarm(uint8_t beat, uint8_t repeat);

// --------------------- Setup & Loop -------------------------
void setup() {
  pinsSetup();

  // Start up the nRF24
  if (!nrf24L01Setup()) {
    // If radio.begin() fails, beep buzzer as an alarm pattern
    alarm(4, 2);
  }

  // Initialize the LED strip and set all LEDs to red
  while (!strip.begin());
  strip.setAllLedsColor(0xFF0000); // All red as an initial state

  // Attach servo on its pin and center it
  servoSetup();
}

void loop() {
  // Check if new data has arrived from the remote
  if (getNrf24L01Data()) {
    clearNrfFlag();
    updateCarActionByNrfRemote();  
    lastNrfUpdateTime = millis();
  }

  // If we haven't received any new data in a while, reset controls
  if (millis() - lastNrfUpdateTime > NRF_UPDATE_TIMEOUT) {
    lastNrfUpdateTime = millis();
    resetNrfDataBuf();
    updateCarActionByNrfRemote(); 
  }
}

// --------------------- Pin & Buzzer Control -----------------
void pinsSetup() {
  // Motor direction & PWM pins
  pinMode(PIN_DIRECTION_LEFT,  OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT,  OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);

  // Line-tracking pins (input mode if used)
  pinMode(PIN_TRACKING_LEFT,   INPUT);
  pinMode(PIN_TRACKING_RIGHT,  INPUT);
  pinMode(PIN_TRACKING_CENTER, INPUT);

  setBuzzer(false); // Make sure buzzer is off initially
}

/*
  setBuzzer: If 'flag' is true, we set pin as OUTPUT and drive it HIGH.
             If 'flag' is false, pin is set to INPUT.
*/
void setBuzzer(bool flag) {
  pinMode(PIN_BUZZER, flag ? OUTPUT : INPUT);
  digitalWrite(PIN_BUZZER, flag ? HIGH : LOW);
}

/*
  alarm: Simple function to beep the buzzer in a pattern (beat times, repeated).
         e.g. alarm(4, 2) = beep beep beep beep, pause, beep beep beep beep
*/
void alarm(uint8_t beat, uint8_t repeat) {
  beat = constrain(beat, 1, 9);
  repeat = constrain(repeat, 1, 255);
  for (int j = 0; j < repeat; j++) {
    for (int i = 0; i < beat; i++) {
      setBuzzer(true);
      delay(100);
      setBuzzer(false);
      delay(100);
    }
    delay(500);
  }
}

// --------------------- Servo -------------------------------
/*
  servoSetup: Attach servo to PIN_SERVO and write it to 90 + offset (center).
  POT1 (0..1023) is mapped to servo angle (0..180) in updateCarActionByNrfRemote().
*/
void servoSetup() {
  servo.attach(PIN_SERVO);
  servo.write(90 + servoOffset); // Center position
}

// --------------------- nRF24 Functions ----------------------
/*
  nrf24L01Setup: Initialize the RF24 radio at 1MBPS, max PA level,
                 with a certain address for both writing & reading.
                 Then we start listening for incoming messages.

  If radio.begin() fails, we can't use the remote.
*/
bool nrf24L01Setup() {
  if (radio.begin()) {
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_1MBPS);
    radio.setRetries(0, 15);           
    radio.openWritingPipe(addresses);  // same address as openReadingPipe
    radio.openReadingPipe(1, addresses);
    radio.startListening();

    // We use FlexiTimer2 to check periodically if data is available
    FlexiTimer2::set(20, 1.0 / 1000, checkNrfReceived);
    FlexiTimer2::start();
    return true;
  }
  return false;
}

/*
  checkNrfReceived: Called by the FlexiTimer2 interrupt every 20 ms.
                    If data is available, read it into nrfDataRead.
*/
void checkNrfReceived() {
  delayMicroseconds(1000); // small delay to ensure no collision
  if (radio.available()) {
    while (radio.available()) {
      radio.read(nrfDataRead, sizeof(nrfDataRead));
    }
    nrfComplete = true;  // Signal that fresh data arrived
    return;
  }
  nrfComplete = false;
}

// Returns 'true' if we have new data from the remote
bool getNrf24L01Data() {
  return nrfComplete;
}

// Clears the 'new data' flag
void clearNrfFlag() {
  nrfComplete = false;
}

/*
  resetNrfDataBuf: If the remote times out, we reset everything to defaults
                   (servo ~512 => centered, joystick ~512 => no drive, etc.)
*/
void resetNrfDataBuf() {
  nrfDataRead[POT1]       = 512; // Mid-range
  nrfDataRead[POT2]       = 0;
  nrfDataRead[JOYSTICK_X] = 512;
  nrfDataRead[JOYSTICK_Y] = 512;
  nrfDataRead[JOYSTICK_Z] = 1;   // "not pressed"
  nrfDataRead[S1]         = 1;
  nrfDataRead[S2]         = 1;
  nrfDataRead[S3]         = 1;
}

// --------------------- Car Drive + Servo --------------------
/*
  updateCarActionByNrfRemote: The main logic that happens whenever new data arrives.
    1) Reads POT1 for servo angle and updates servo.
    2) Reads JOYSTICK_X, JOYSTICK_Y for motor drive.
    3) Reads JOYSTICK_Z for buzzer on/off.
*/
void updateCarActionByNrfRemote() {
  //
  // 1) Update servo from POT1
  //
  int pot1Value  = nrfDataRead[POT1]; // 0..1023 from the remote
  // Map to servo angle 0..180 (adjust as needed)
  int servoAngle = map(pot1Value, 0, 1023, 0, 180);
  servoAngle = constrain(servoAngle + servoOffset, 0, 180);
  servo.write(servoAngle);

  //
  // 2) Drive motors from JOYSTICK_X, JOYSTICK_Y
  //
  int x = nrfDataRead[JOYSTICK_X] - 512;  // -512..+511
  int y = nrfDataRead[JOYSTICK_Y] - 512;  // -512..+511

  // Basic differential drive "mixing"
  int pwmL, pwmR;
  if (y < 0) {
    // Forward drive
    pwmL = (-y + x) / 2;
    pwmR = (-y - x) / 2;
  } else {
    // Reverse drive
    pwmL = (-y - x) / 2;
    pwmR = (-y + x) / 2;
  }
  motorRun(pwmL, pwmR);

  //
  // 3) Buzzer from JOYSTICK_Z
  //
  // Joystick Z (button) is in [4]; 0 => pressed => beep
  if (nrfDataRead[JOYSTICK_Z] == 0) {
    setBuzzer(true);
  } else {
    setBuzzer(false);
  }
}

/*
  motorRun: Takes the two motor "speed" commands (pwmL, pwmR), determines direction
            bits, applies constraints and speed reduction, then writes them via
            analogWrite() for PWM control.
*/
void motorRun(int speedl, int speedr) {
  // 1) Decide direction bits for each side (0 or 1).
  int dirL = 0;
  int dirR = 0;

  if (speedl > 0) {
    dirL = 0 ^ MOTOR_DIRECTION;  // ^ = XOR
  } else {
    dirL = 1 ^ MOTOR_DIRECTION;
    speedl = -speedl;           // convert speed to positive
  }
  if (speedr > 0) {
    dirR = 1 ^ MOTOR_DIRECTION;
  } else {
    dirR = 0 ^ MOTOR_DIRECTION;
    speedr = -speedr;
  }

  // 2) Limit speeds to [0..255] before scaling
  speedl = constrain(speedl, 0, 255);
  speedr = constrain(speedr, 0, 255);

  // 3) Constrain speed_reduction and apply it
  //    This limits our top speed to speed_reduction * 255.
  speed_reduction = constrain(speed_reduction, 0.6, 1.0);
  speedl = (int)(speedl * speed_reduction);
  speedr = (int)(speedr * speed_reduction);

  // 4) If both speeds are under the DEAD zone, set them to 0
  if (speedl < MOTOR_PWM_DEAD && speedr < MOTOR_PWM_DEAD) {
    speedl = 0;
    speedr = 0;
  }

  // 5) Output direction pins and PWM signals to the motors
  digitalWrite(PIN_DIRECTION_LEFT,  dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT,   speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT,  speedr);
}
