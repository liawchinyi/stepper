#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "DRV8711.h"

// Pin Definitions
#define RESET_PIN PB0
#define SLEEPN PB1
#define STEP_PIN PA4
#define DIR_PIN PA3
#define GREEN_LED PC13
#define RED_LED PA8
#define LIMIT_SWITCH_PIN PA1  // Define your limit switch pin
#define CS_PIN PA15           // Define Chip Select pin
#define DEBOUNCE_DELAY 50     // 50 milliseconds debounce delay

uint8_t rxbytes[8];
const long maxPosition = 52000;      // Maximum position (100,000 steps)
long currentPosition = maxPosition;  // Current position of the stepper motor
const int maxSpeed = 1000;           // Maximum speed in steps per second
const int acceleration = 500;        // Acceleration in steps per second^2
int status = 0;

// Define an alternative SPI interface
SPIClass etx_spi(PB5, PB4, PB3);  // MOSI, MISO, SCLK

// Create a DRV8711 instance using the alternative SPI interface
DRV8711 drv8711(etx_spi, CS_PIN);
HardwareSerial Serial3(PB11, PB10);
HardwareSerial Serial2(PA10, PA9);

long targetPosition = 0;
long newPosition = 0;
bool moving = false;
long stepsToMove = 0;
bool direction = false;
long accelDist = 0;
long currentStep = 0;
long speed = 0;
long stepDelay = 0;
long lastWaitTime = 0;
long currentWaitTime = 0;
long WaitDelay = 100;
bool greenLedState = false;  // Variable to keep track of the LED state
bool redLedState = false;    // Variable to keep track of the LED state

// Function to move the stepper motor to a specified position
void moveToPosition(long position) {
  if (position < 0) position = 0;                      // Ensure position is not negative
  if (position > maxPosition) position = maxPosition;  // Ensure position does not exceed maxPosition

  targetPosition = position;
  stepsToMove = targetPosition - currentPosition;
  direction = stepsToMove > 0;
  digitalWrite(DIR_PIN, direction ? LOW : HIGH);
  stepsToMove = abs(stepsToMove);
  accelDist = (long)pow(maxSpeed, 2) / (2 * acceleration);  // Distance needed to accelerate/decelerate
  currentStep = 0;
  speed = 0;
  moving = true;
  if (stepsToMove != 0) {
    digitalWrite(SLEEPN, HIGH);
    delay(100);  //When exiting from sleep mode, some time (approximately 1 mS) amount of time (in milliseconds) specified as parameter.
    Serial2.printf("moveToPosition(%d) \n", position);
  }
}

void setup() {
  Serial2.begin(115200);
  Serial3.begin(115200);

  while (!Serial2) {};
  delay(1000);  // Delay for USB initialization

  HardwareTimer *MyTim3 = new HardwareTimer(TIM3);
  HardwareTimer *MyTim2 = new HardwareTimer(TIM2);

  MyTim3->setOverflow(30000, MICROSEC_FORMAT);  // 30 milliseconds
  MyTim3->attachInterrupt(TIM3_IT_callback);
  MyTim3->resume();  // Start Timer Interrupt

  MyTim2->setOverflow(10, MICROSEC_FORMAT);  // 10 microseconds
  MyTim2->attachInterrupt(TIM2_IT_callback);
  MyTim2->resume();  // Start Timer Interrupt

  // Initialize pin modes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  pinMode(SLEEPN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Reset DRV8711
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
  digitalWrite(RESET_PIN, LOW);
  //On exiting reset state, some time (approximately 1 mS) needs to pass before the part is fully functional.
  delay(100);
  digitalWrite(SLEEPN, HIGH);
  delay(10);
  digitalWrite(DIR_PIN, LOW);

  // Initialize DRV8711
  drv8711.begin(DRV8711_FULL, 0x02);
  drv8711.enable_motor();  // Enable motor driver
  drv8711.clear_status();  // Clear status
  delay(10);               //amount of time (in milliseconds)

  // Print the configured values
  Serial2.printf("CTRL: 0x%x\n", drv8711.get_reg(0x00));
  Serial2.printf("TORQUE: 0x%x\n", drv8711.get_reg(0x01));
  Serial2.printf("OFF: 0x%x\n", drv8711.get_reg(0x02));
  Serial2.printf("BLANK: 0x%x\n", drv8711.get_reg(0x03));
  Serial2.printf("DECAY: 0x%x\n", drv8711.get_reg(0x04));
  Serial2.printf("STALL: 0x%x\n", drv8711.get_reg(0x05));
  Serial2.printf("DRIVE: 0x%d\n", drv8711.get_reg(0x06));
  Serial2.printf("STATUS: 0x%x\n", drv8711.get_reg(0x07));

  moveToPosition(0);
}

void loop() {
  if (Serial2.available()) {
    rxbytes[0] = Serial2.read();
    // Command Decoder
    switch (rxbytes[0]) {
      case '1':
        newPosition = 0;
        break;
      case '2':  // Set Analogue Output 1
        newPosition = 15000;
        break;
      case '3':
        newPosition = 30000;
        break;
      case '4':
        newPosition = 50000;
        break;
      default:
        // statements
        break;
    }
    moveToPosition(newPosition);

    redLedState = !redLedState;                       // Toggle the LED state
    digitalWrite(RED_LED, redLedState ? HIGH : LOW);  // Set the LED state
  }
  /*
  else
  {
    if(moving == false && currentPosition == 0)
    moveToPosition(50000);   
      if(moving == false && currentPosition == 50000)
    moveToPosition(0);   
  }
  */

  // Toggle the LED state
  greenLedState = !greenLedState;
  digitalWrite(GREEN_LED, greenLedState ? HIGH : LOW);

  // Print the status of the DRV8711
  status = drv8711.get_status();
  if (status != 0) {
    Serial2.printf("CTRL: 0x%x, STATUS_REG 0x%x, CurrentPOS %d, \n", drv8711.get_reg(0x00), status, currentPosition);
    drv8711.clear_status();  // Clear status
  }
}

void TIM3_IT_callback(void) {
  uint8_t mappedPosition = map(currentPosition, 0, maxPosition, 0, 255);
  //Serial2.println(currentPosition);
}

void TIM2_IT_callback(void) {
  static unsigned long lastStepTime = 0;

  if (moving) {
    // Read the state of the limit switch
    int limitSwitchState = digitalRead(LIMIT_SWITCH_PIN);

    // If limit switch is activated and the direction is not away from the limit switch, stop the motor
    if (limitSwitchState == HIGH && !direction) {
      moving = false;

      Serial2.printf("Limit switch activated. Stopping motor.\n");
      currentPosition = 0;
    } else {
      // Calculate speed based on acceleration profile
      if (currentStep < accelDist) {
        speed = sqrt(2 * acceleration * currentStep);
      } else if (currentStep >= stepsToMove - accelDist) {
        speed = sqrt(2 * acceleration * (stepsToMove - currentStep));
      } else {
        speed = maxSpeed;
      }

      // Limit speed to maxSpeed
      if (speed > maxSpeed) {
        speed = maxSpeed;
      }

      // Calculate step delay
      stepDelay = 200000 / speed;

      // Perform step if enough time has passed
      unsigned long currentTime = micros();
      if (currentTime - lastStepTime >= stepDelay) {
        lastStepTime += stepDelay;  // Update lastStepTime for precise timing
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(stepDelay / 2);  // Half-step pulse
        digitalWrite(STEP_PIN, LOW);

        // Update position and step count
        currentStep++;
        currentPosition += direction ? 1 : -1;

        // Check if movement is complete
        if (currentStep >= stepsToMove) {
          moving = false;
          currentStep = 0;
        }
      }
    }
  } else {
    currentWaitTime = micros();
    if (currentWaitTime - lastWaitTime >= WaitDelay) {
      lastWaitTime += WaitDelay;  // Update lastStepTime for precise timing
      digitalWrite(SLEEPN, LOW);
    }
  }
}
