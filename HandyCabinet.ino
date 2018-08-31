#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <EEPROM.h>

// Pins
#define MIC_RX_PIN 0
#define MIC_TX_PIN 1
#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4
#define MOTOR_PIN 5
#define SW_PIN 6
#define RESET_PIN 7
#define V_X_PIN 1
#define V_Y_PIN 2

// Voice module
#define HEADER 0xAA
//// Keys
#define WAITING_MODE 0x00
#define COMPACT_MODE 0x37
#define GROUP1 0x21
#define GROUP2 0x22
#define GROUP3 0x23
#define COMMAND1 0x11 // Cabinet In
#define COMMAND2 0x12 // Cabinet In
#define COMMAND3 0x13 // Cabinet Out
#define COMMAND4 0x14 // Cabinet Out 
#define COMMAND5 0x15 // Cancel
//// Simple vars
#define COMMAND_WAIT_TIME 5000
#define DELAY_TIME 300

// Cabinet
#define MOVE_TIME_OUT 2500
#define MOVE_TIME_IN 2690
#define ADDRESS_OF_STATE 1
#define DEFAULT_STATE false // false is for the cabinet being in and true for it being out.

// Motor
#define M_FORWARD_SPEED -32
#define M_BACKWARD_SPEED 32
#define M_STOP_SPEED 0

// Joystick
#define RESTING 512
#define JOYSTICK_THRESHOLD 500 // Can be an integer between 0 - 512 (both ends inclusive)

// etc
#define BLINK_TIME 1000
#define DELAY_BEFORE_RESET 500

SoftwareSerial SWSerial(NOT_A_PIN, MOTOR_PIN);
SabertoothSimplified ST(SWSerial);

void setup()
{
  Serial.begin(9600);
  SWSerial.begin(9600);

  pinMode(SW_PIN, INPUT_PULLUP);
  if (digitalRead(SW_PIN) == LOW) // Check if user is holding down on the joystick while
  {                               // the arudino is being set up
    EEPROM.write(ADDRESS_OF_STATE, DEFAULT_STATE);
    color(HIGH, HIGH, HIGH, BLINK_TIME); // Blink white light to confirm state reset
  }

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  digitalWrite(RESET_PIN, HIGH);
  pinMode(RESET_PIN, OUTPUT);

  pinMode(MIC_TX_PIN, INPUT);
  sendCommand(COMPACT_MODE);
  sendCommand(GROUP1);

  Serial.flush();
}

void loop()
{
  colorOff();
  if (Serial.available())
  {
    byte input = Serial.read();
    if (input == COMMAND1 || input == COMMAND2) // If Saul said his activation command, or
    {                                           // whoever's voice was recorded for commands 1 and 2 in group 1.
      colorOn(HIGH, HIGH, HIGH);
      sendCommand(GROUP2);
      colorOff();
      doCommand(getCommand());
    }
    else if (input == COMMAND3 || input == COMMAND4) // If Leme said her activation command, or
    {                                                // whoever's voice was recorded for commands 3 and 4 in group 1.
      colorOn(HIGH, HIGH, HIGH);
      sendCommand(GROUP3);
      colorOff();
      doCommand(getCommand());
    }
  }
  checkJoystick();
  colorOn(LOW, LOW, HIGH);
}

int sendCommand(byte hex)
{
  Serial.write(HEADER); // Go to waiting mode first
  Serial.write(byte(WAITING_MODE));
  delay(DELAY_TIME);
  Serial.flush();
  Serial.write(HEADER); // Then send command
  Serial.write(hex);
  delay(DELAY_TIME);
  return Serial.read(); // Return response
}

void reset()
{
  Serial.flush();
  delay(DELAY_BEFORE_RESET);
  digitalWrite(RESET_PIN, LOW);
}

int getCommand()
{
  int in = -1;
  unsigned long startTime = millis();
  while (millis() <= startTime + COMMAND_WAIT_TIME && !isValidCommand(in))
    if (Serial.available())
      in = Serial.read();
  return in;
}

void doCommand(int in)
{
  if (isValidCommand(in))
  {
    color(LOW, HIGH, LOW, BLINK_TIME);
    doAction(in);
  }
  else color(HIGH, LOW, LOW, BLINK_TIME);
  reset(); // The reset is needed because the voice module fails to listen again unless reseted
  sendCommand(GROUP1);
}

void color(byte red, byte green, byte blue, unsigned long blinkTime)
{
  colorOn(red, green, blue);
  delay(blinkTime);
  colorOff();
}

void colorOn(byte red, byte green, byte blue)
{
  digitalWrite(RED_PIN, red);
  digitalWrite(GREEN_PIN, green);
  digitalWrite(BLUE_PIN, blue);
}

void colorOff()
{
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

bool isValidCommand(int choice)
{
  switch (choice)
  {
    case (COMMAND1):
      return true;
      break;
    case (COMMAND2):
      return true;
      break;
    case (COMMAND3):
      return true;
      break;
    case (COMMAND4):
      return true;
      break;
    case (COMMAND5):
      return true;
      break;
    default:
      return false;
      break;
  }
}

void doAction(int choice)
{
  switch (choice)
  {
    case (COMMAND1):
      moveCabinetIn();
      break;
    case (COMMAND2):
      moveCabinetIn();
      break;
    case (COMMAND3):
      moveCabinetOut();
      break;
    case (COMMAND4):
      moveCabinetOut();
      break;
    case (COMMAND5):
      cancel();
      break;
  }
}

void checkJoystick()
{
  if (!isJoystickResting())
  {
    color(LOW, HIGH, LOW, BLINK_TIME);
    if (isOut())
      moveCabinetIn();
    else
      moveCabinetOut();
  }
}

bool isOut()
{
  return EEPROM.read(ADDRESS_OF_STATE) != false;
}

void flipState()
{
  EEPROM.write(ADDRESS_OF_STATE, !isOut());
}

bool isJoystickResting()
{
  int readingX = getJoystickReadingX();
  int readingY = getJoystickReadingY();
  return isWithinJoystickRange(readingX) && isWithinJoystickRange(readingY);
}

bool isWithinJoystickRange(int reading)
{
  return RESTING - JOYSTICK_THRESHOLD <= reading &&
         RESTING + JOYSTICK_THRESHOLD >= reading;
}

int getJoystickReadingX()
{
  return analogRead(V_X_PIN);
}

int getJoystickReadingY()
{
  return analogRead(V_Y_PIN);
}

void moveCabinetIn()
{
  if (!isOut()) return;

  unsigned long startTime = millis();
  while (millis() < startTime + MOVE_TIME_IN)
    moveBackward();
  stopMotor();
  flipState();
}

void moveCabinetOut()
{
  if (isOut()) return;

  unsigned long startTime = millis();
  while (millis() < startTime + MOVE_TIME_OUT)
    moveForward();
  stopMotor();
  flipState();
}

void moveBackward()
{
  ST.motor(M_BACKWARD_SPEED);
}

void moveForward()
{
  ST.motor(M_FORWARD_SPEED);
}

void stopMotor()
{
  ST.motor(M_STOP_SPEED);
}

void turnLightsOn()
{
  // Turn the lights on
}

void turnLightsOff()
{
  // Turn the lights off
}

void cancel() // Dummy method
{
  return;
}

