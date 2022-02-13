#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>
#include <EEPROM.h>
#include "Wire.h"
#include "BluetoothSerial.h"

// Debug flags
#define DEBUG_SERIAL_MESSAGES
#define DEBUG_BT_DECODING

#define BLUETOOTH_NAME "Szopka Bluetooth"
#define SETTINGS_TABLE_SIZE 256
#define BLUETOOTH_TIMEOUT (uint32_t)600 // in seconds
#define SWITCH_OFF 0
#define SWITCH_ON 1

// PIN configuration
#define BUTTON_PIN 13
#define MOTOR_1_PIN 12
#define MOTOR_2_PIN 14
#define MOTOR_3_PIN 27
#define AMOUNT_OF_MOTORS 3

#define MCP1_ADDR 0x20
#define MCP2_ADDR 0x21

#define ERROR 1
#define NO_ERROR 0
#define EEPROM_MAX_SIZE 4096

#define ATOI_ERROR 0

// Possible decoding errors
#define ERROR_LOOP_TIME_DECODING (-1)
#define ERROR_SOCKET_ID_DECODING (-2)
#define ERROR_START_TIME_DECODING (-3)
#define ERROR_DURATION_DECODING (-4)
#define ERROR_MAX_LINE_CHARACTERS_EXCEEDED (-5)
#define ERROR_INPROPER_PLACE_OF_END_MARKER (-6)
#define ERROR_UNKNOWN_STARTER_MARKER (-7)

/**
 * @brief holding current state of ECU 
 * STATES description below
 *  INIT - state where ECU starts
 *   |     after finishing setup() settings go to RUNTIME
 *   v
 * RUNTIME - state where ECU handles relays (nativity scene is working)
 *  | ^      when button pressed change state to RUNMTIME or SETTINGS according to its previous state
 *  v |      when state was SETTINGS and bluetooth timer expired set it to RUNTIME (case to prevent unattended bt connection)
 * SETTINGS - state where ECU starts bluetooth and awaits for settings received on bluetooth 
 */
enum ECU_Mode
{
  INIT = 0,
  SETTINGS,
  RUNTIME
} currentECUMode;

typedef struct
{
  uint8_t socketID;
  uint16_t startTime;
  uint16_t duration;
} SettingsStruct;

SettingsStruct settingsTable[SETTINGS_TABLE_SIZE];

uint32_t loopTime = 10;

bool isBluetoothActive = false;

int previousButtonState = HIGH;
BluetoothSerial bluetooth;
Adafruit_MCP23X17 mcp1; // handles pins 0-15
Adafruit_MCP23X17 mcp2; // handles pins 16-31

// Timer part
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

// String decoding
#define MAX_CHARS_IN_LINE 100
char receivedString[MAX_CHARS_IN_LINE];
bool isReceivingInProgress = false;
bool isFirstLine = false;
int settingsIndex = 0;
int charIndex = 0;
int tableIndex = 0;
const char *delimter = ",";

// Functions prototypes
void setup();
void loop();
void handleSettingsMode();
void handleRuntimeMode();
void handleNativitySceneRelays(int PIN_Number, bool switchType);
void handleMotors();
void initMCPs();
void initMotors();
void stopRelayControl();
void setup_1s_timer();
void IRAM_ATTR onTimer();
uint32_t check_1s_TimerIteration();
void freeTimer();
void resetTimer();
void checkButtonState();
void checkifBluetoothShouldBeActive();
bool checkIfBluetoothTimerHasExpired();
bool checkNativitySceneTimer();
int recvWithStartEndMarkers(char receivedChar);
void cleanCurrentSettings();
void cleanString();
void cleanReceptionVars();
void serialDecodingFallbackScenario();
int decodeReceivedData();
int decodeFirstLine();
int decodeRelayTimingSetting();
void readFromNVM();
void writeToNVM();
bool checkEEPROMSize();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }
#ifdef DEBUG_SERIAL_MESSAGES
  Serial.println("Init started");
#endif

  initMCPs();

  // init bluetooth on/off button
  pinMode(BUTTON_PIN, INPUT);

  // init motor pinouts
  initMotors();

  setup_1s_timer();
  readFromNVM();

  delay(50); // delete it later
#ifdef DEBUG_SERIAL_MESSAGES
  Serial.println("Init finished");
#endif
  currentECUMode = RUNTIME;
}

/*********************
 * @brief MAIN LOOP
 * 
 ********************/
void loop()
{
  // check if button was pressed
  checkButtonState();
  // if it was pressed change ECU mode,
  // or change mode when bluetooth was on and its timer has expired
  checkifBluetoothShouldBeActive();

  if (currentECUMode == SETTINGS)
  {
    // mainly handles bluetooth
    handleSettingsMode();
  }
  else if (currentECUMode == RUNTIME)
  {
    // handles switching of relays
    handleRuntimeMode();
  }
}

/* Function to initialize motor pins and set it to off by default */
void initMotors()
{
  pinMode(MOTOR_1_PIN, OUTPUT);
  digitalWrite(MOTOR_1_PIN, HIGH);

  pinMode(MOTOR_2_PIN, OUTPUT);
  digitalWrite(MOTOR_1_PIN, HIGH);

  pinMode(MOTOR_3_PIN, OUTPUT);
  digitalWrite(MOTOR_1_PIN, HIGH);
}

void handleSettingsMode()
{
  if (bluetooth.available())
  {
    int retVal = 0;
    //decode bt here
    char receivedChar = bluetooth.read();
    retVal = recvWithStartEndMarkers(receivedChar);

#ifdef DEBUG_BT_DECODING
    Serial.print("DECODE ReceivedChar=");
    Serial.print(receivedChar);
    Serial.print(" | retVal=");
    Serial.print(retVal);
    Serial.print("\n");
#endif
  }
}

void handleRuntimeMode()
{
  // if 1s timer fired (1s has passed) take an action
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
  {
    uint32_t currentTimerIteration = 0;
    // Read the interrupt count in critical section
    portENTER_CRITICAL(&timerMux);
    currentTimerIteration = isrCounter;
    portEXIT_CRITICAL(&timerMux);

#ifdef DEBUG_SERIAL_MESSAGES
    Serial.print("onTimer num of iteration: ");
    Serial.print(currentTimerIteration);
    Serial.print("\n");
#endif

    // Iterate over table setting to perfrom action
    for (int i = 0; i < SETTINGS_TABLE_SIZE; i++)
    {
      if (settingsTable[i].startTime == currentTimerIteration)
      {
        handleNativitySceneRelays(settingsTable[i].socketID, SWITCH_ON);
      }
      else if ((settingsTable[i].startTime + settingsTable[i].duration) == currentTimerIteration)
      {
        handleNativitySceneRelays(settingsTable[i].socketID, SWITCH_OFF);
      }
    }

    // Reset timer when nativity scene ends its loop
    if (currentTimerIteration >= loopTime)
    {
      resetTimer();
    }
  }
}

void handleNativitySceneRelays(int PIN_Number, bool switchType)
{
  // on each MCP there are only 16 PINS, checks where to send
  // Wemos D1 R32 handles motor 0-2 pins, 3 in total
  // mcp1 handles 3-18 pins (sockets), 16 in total
  // mcp2 handles 19-34 pins (sockets), 16 in total
  if (PIN_Number < AMOUNT_OF_MOTORS)
  {
    handleMotors(PIN_Number, switchType);
  }
  else if (PIN_Number < 16 + AMOUNT_OF_MOTORS)
  {
    if (switchType == SWITCH_ON)
    {
      mcp1.digitalWrite(PIN_Number - AMOUNT_OF_MOTORS, LOW);
    }
    else
    {
      mcp1.digitalWrite(PIN_Number - AMOUNT_OF_MOTORS, HIGH);
    }
  }
  else
  {
    if (switchType == SWITCH_ON)
    {
      mcp2.digitalWrite((PIN_Number - 16) - AMOUNT_OF_MOTORS, LOW);
    }
    else
    {
      mcp2.digitalWrite((PIN_Number - 16) - AMOUNT_OF_MOTORS, HIGH);
    }
  }
#ifdef DEBUG_SERIAL_MESSAGES
  Serial.print("Socket number: ");
  Serial.print(PIN_Number);
  Serial.print(" is set to ");
  if (switchType == SWITCH_ON)
    Serial.print("ON");
  else
    Serial.print("OFF");
  Serial.print("\n");
#endif
}

/* Handles digital writes to motors */
void handleMotors(int PIN_Number, bool switchType)
{
  switch (PIN_Number)
  {
  case 0:
    if (switchType == SWITCH_ON)
      digitalWrite(MOTOR_1_PIN, LOW);
    else
      digitalWrite(MOTOR_1_PIN, HIGH);
    break;

  case 1:
    if (switchType == SWITCH_ON)
      digitalWrite(MOTOR_2_PIN, LOW);
    else
      digitalWrite(MOTOR_2_PIN, HIGH);
    break;

  case 2:
    if (switchType == SWITCH_ON)
      digitalWrite(MOTOR_3_PIN, LOW);
    else
      digitalWrite(MOTOR_3_PIN, HIGH);
    break;

  default:
    // no such case
    break;
  }
}

void initMCPs()
{
  while (!mcp1.begin_I2C(MCP1_ADDR))
  {
    Serial.println("Could not started communication with MCP1");
  }
  while (!mcp2.begin_I2C(MCP2_ADDR))
  {
    Serial.println("Could not started communication with MCP2");
  }
  stopRelayControl();
}

void stopRelayControl()
{
  for (int i = 0; i < 16; i++)
  {
    mcp1.pinMode(i, OUTPUT);
    mcp1.digitalWrite(i, HIGH);
    mcp2.pinMode(i, OUTPUT);
    mcp2.digitalWrite(i, HIGH);
  }
}

void setup_1s_timer()
{
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void IRAM_ATTR onTimer()
{
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

uint32_t check_1s_TimerIteration()
{
  uint32_t isrCount = 0;
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
  {
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    portEXIT_CRITICAL(&timerMux);
#ifdef DEBUG_SERIAL_MESSAGES
    // Print it
    Serial.print("onTimer num of iteration: ");
    Serial.print(isrCount);
    Serial.print("\n");
#endif
  }
  return isrCount;
}

void freeTimer()
{
  if (timer)
  {
    // Stop and free timer
    timerEnd(timer);
    timer = NULL;
    isrCounter = 0;
  }
}

void resetTimer()
{
  // free the old timer and start new one
  freeTimer();
  setup_1s_timer();
}

/**
 * @brief Function to detect button pressed and do its actions
 * button 1st pressed - start bluetooth
 * button 2nd pressed - end bluetooth
 */
void checkButtonState()
{
  //Serial.println("checking button state");
  int buttonState = digitalRead(BUTTON_PIN);
  /*Serial.print("prevState = ");
  Serial.print(previousButtonState);
  Serial.print(", currState = ");
  Serial.print(buttonState);
  Serial.print("\n");*/
  if (buttonState == HIGH && buttonState != previousButtonState)
  {
    previousButtonState = buttonState;
    //change bluetooth state
    if (!isBluetoothActive)
      isBluetoothActive = true;
    else
      isBluetoothActive = false;
  }
  else if (buttonState == LOW && buttonState != previousButtonState)
  {
    previousButtonState = buttonState;
  }
}
/**
 * @brief Function beginning and ending bluetooth
 * isBluetoothActive = true - bluetooth on
 * isBluetoothActive = false - bluetooth off
 * timer expired - bluetooth off
 */
void checkifBluetoothShouldBeActive()
{
  if (isBluetoothActive && currentECUMode == RUNTIME)
  {
    bluetooth.begin(BLUETOOTH_NAME);
    resetTimer();
    stopRelayControl();
#ifdef DEBUG_SERIAL_MESSAGES
    Serial.println("Bluetooth switched ON");
#endif
    currentECUMode = SETTINGS;
  }
  else if ((!isBluetoothActive && currentECUMode == SETTINGS) ||
           // if timer has expired switch off bluetooth
           (isBluetoothActive && checkIfBluetoothTimerHasExpired() && currentECUMode == SETTINGS))
  {
    bluetooth.end();
    resetTimer();
#ifdef DEBUG_SERIAL_MESSAGES
    Serial.println("Bluetooth switched OFF");
#endif
    currentECUMode = RUNTIME;
    // for the second part of previous if statement
    if (isBluetoothActive)
      isBluetoothActive = false;
  }
}

bool checkIfBluetoothTimerHasExpired()
{
  if (check_1s_TimerIteration() > BLUETOOTH_TIMEOUT)
    return true;
  else
    return false;
}

int recvWithStartEndMarkers(char receivedChar)
{
  char startMarker = '<';
  char endMarker = '>';
  char endOfLineMarker = ';';
  int retVal = 0;

  if (isReceivingInProgress == true)
  {
    if (receivedChar == endOfLineMarker)
    {
      receivedString[charIndex] = '\0'; // terminate the string
      retVal = decodeReceivedData();
      cleanReceptionVars();
    }
    else if (receivedChar == endMarker)
    {
      if (receivedString[0] != '\0')
      {
        retVal = ERROR_INPROPER_PLACE_OF_END_MARKER;
      }
      isFirstLine = false;
      isReceivingInProgress = false;
      cleanReceptionVars();
      writeToNVM();
    }
    else if (receivedChar != endOfLineMarker)
    {
      receivedString[charIndex] = receivedChar;
      charIndex++;
      if (charIndex >= MAX_CHARS_IN_LINE)
      {
        serialDecodingFallbackScenario();
        // Exceeded max line characters
        retVal = ERROR_MAX_LINE_CHARACTERS_EXCEEDED;
      }
    }
  }
  else if (receivedChar == startMarker && isReceivingInProgress == false)
  {
    cleanCurrentSettings();
    isReceivingInProgress = true;
    isFirstLine = true;
  }
  else
  {
    retVal = ERROR_UNKNOWN_STARTER_MARKER;
  }

  return retVal;
}

void writeToNVM()
{
  if (checkEEPROMSize() == NO_ERROR)
  {
    EEPROM.put(0, loopTime);                     // first bytes contains looptime
    EEPROM.put(sizeof(loopTime), settingsTable); // the rest contains settingsTable
    EEPROM.commit();
  }
}

void readFromNVM()
{
  if (checkEEPROMSize() == NO_ERROR)
  {
    uint32_t nvm_size = sizeof(settingsTable) + sizeof(loopTime);
    EEPROM.begin(nvm_size);
    EEPROM.get(0, loopTime);
    EEPROM.get(sizeof(loopTime), settingsTable);
  }
}

bool checkEEPROMSize()
{
  uint32_t nvm_size = sizeof(settingsTable) + sizeof(loopTime);
  if (nvm_size > EEPROM_MAX_SIZE)
  {
    Serial.print("sizeNeeded = ");
    Serial.print(nvm_size);
    Serial.print(", max size allowed = ");
    Serial.print(EEPROM_MAX_SIZE);
    Serial.print("\n");
    Serial.println("MAX EEPROM Size Exceeded!");
    return ERROR;
  }
  else
    return NO_ERROR;
}

void cleanCurrentSettings()
{
  settingsIndex = 0;
  for (int i = 0; i < SETTINGS_TABLE_SIZE; i++)
  {
    settingsTable[i].socketID = 0;
    settingsTable[i].startTime = 0;
    settingsTable[i].duration = 0;
  }
}

void cleanString()
{
  for (int i = 0; i < MAX_CHARS_IN_LINE; i++)
  {
    receivedString[i] = '\0';
  }
}

void cleanReceptionVars()
{
  charIndex = 0;
  cleanString();
}

void serialDecodingFallbackScenario()
{
  charIndex = 0;
  cleanString();
  isFirstLine = false;
  isReceivingInProgress = false;
}

int decodeReceivedData()
{ // split the data into its parts

  int retVal = 0;

  // first line contains loopTime only
  if (isFirstLine)
  {
    retVal = decodeFirstLine();
  }
  else
  {
    retVal = decodeRelayTimingSetting();
  }

  return retVal;
}

int decodeFirstLine()
{
  int retVal = NO_ERROR;
  int temp = atoi(receivedString);
  if (temp != ATOI_ERROR)
  {
    loopTime = temp;
    isFirstLine = false;
  }
  else
  {
    // loop time cannot be 0 or it is not a number
    serialDecodingFallbackScenario();
    retVal = ERROR_LOOP_TIME_DECODING;
  }
  return retVal;
}

int decodeRelayTimingSetting()
{
  int retVal = 0;
  char *strtokIndx = NULL; // this is used by strtok() as an index
  int socketID = 0;
  int startTime = 0;
  int duration = 0;

  // Decode first element (socketID)
  strtokIndx = strtok(receivedString, delimter); // starts
  if (*strtokIndx == '0')                        // socketID can be 0
  {
    socketID = 0;
  }
  else
  {
    socketID = atoi(strtokIndx); // convert this part to an integer
    if (socketID == ATOI_ERROR)
    {
      return ERROR_SOCKET_ID_DECODING;
    }
  }

  // Decode second element (startTime)
  strtokIndx = strtok(NULL, delimter); // this continues where the previous call left off
  if (*strtokIndx == '0')              // startTime can be 0
  {
    startTime = 0;
  }
  else
  {
    startTime = atoi(strtokIndx); // convert this part to an integer
    if (startTime == ATOI_ERROR)
    {
      return ERROR_START_TIME_DECODING;
    }
  }

  // Decode third element (duration)
  strtokIndx = strtok(NULL, delimter); // this continues where the previous call left off
  duration = atoi(strtokIndx);         // duration cannot be 0, no need to check it for containing "0"
  if (duration == ATOI_ERROR)
  {
    return ERROR_DURATION_DECODING;
  }

  // if there was no error parse final data
  if (retVal == 0)
  {
    settingsTable[settingsIndex].socketID = socketID;
    settingsTable[settingsIndex].startTime = startTime;
    settingsTable[settingsIndex].duration = duration;
    settingsIndex++;
  }

  return retVal;
}