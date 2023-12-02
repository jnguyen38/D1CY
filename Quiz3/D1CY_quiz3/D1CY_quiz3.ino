// =======================================================================================
//                     PS3 Starting Sketch for Notre Dame Droid Class
// =======================================================================================
//                          Last Revised Date: 01/08/2023
//                             Revised By: Prof McLaughlin
// =======================================================================================
// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------
#include <PS3BT.h>
#include <usbhub.h>
#include <Sabertooth.h>
#include <Adafruit_TLC5947.h>
#include <MP3Trigger.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>

// ---------------------------------------------------------------------------------------
//                             OLED Display Initialization
// ---------------------------------------------------------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------------------------------------------------------------------------------
//                 Setup for USB, Bluetooth Dongle, & PS3 Controller
// ---------------------------------------------------------------------------------------
USB Usb;
BTD Btd(&Usb);
PS3BT *PS3Controller = new PS3BT(&Btd);

Servo myServo;
MP3Trigger MP3Trigger;

// ---------------------------------------------------------------------------------------
//    Used for PS3 Fault Detection
// ---------------------------------------------------------------------------------------
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;

boolean isPS3ControllerInitialized = false;
boolean mainControllerConnected = false;
boolean WaitingforReconnect = false;
boolean isFootMotorStopped = true;

// ---------------------------------------------------------------------------------------
//    Drive Variables
// ---------------------------------------------------------------------------------------
int driveDeadBandRange = 10;
byte joystickDeadZoneRange = 15;
#define SABERTOOTH_ADDR   128
Sabertooth *ST = new Sabertooth(SABERTOOTH_ADDR, Serial1);    // TX1 - Pin #18

int currentSpeed = 0;
int currentTurn = 0;
boolean droidMoving = false;
int drivingStart; // Stores when the droid started driving
int drivingStop; // Store when the droid stopped driving

// ---------------------------------------------------------------------------------------
//    Used for PS3 Controller Request Management
// ---------------------------------------------------------------------------------------
long previousRequestMillis = millis();
boolean extraRequestInputs = false;

// ---------------------------------------------------------------------------------------
//    Request State Machine Variables for PS3 Controller
// ---------------------------------------------------------------------------------------

// Main state varable to determine if a request has been made by the PS3 Controller
boolean reqMade = false;
boolean reqLeftJoyMade = false;
boolean reqRightJoyMade = false;

// LEFT & RIGHT Joystick State Request Values
boolean reqLeftJoyUp = false;
boolean reqLeftJoyDown = false;
int reqLeftJoyYValue = 0;

boolean reqLeftJoyLeft = false;
boolean reqLeftJoyRight = false;
int reqLeftJoyXValue = 0;

boolean reqRightJoyUp = false;
boolean reqRightJoyDown = false;
int reqRightJoyYValue = 0;

boolean reqRightJoyLeft = false;
boolean reqRightJoyRight = false;
int reqRightJoyXValue = 0;

// PS3 Controller Button State Variables
boolean reqArrowUp = false;
boolean reqArrowDown = false;
boolean reqArrowLeft = false;
boolean reqArrowRight = false;
boolean reqCircle = false;
boolean reqCross = false;
boolean reqTriangle = false;
boolean reqSquare = false;
boolean reqL1 = false;
boolean reqL2 = false;
boolean reqR1 = false;
boolean reqR2 = false;
boolean reqSelect = false;
boolean reqStart = false;
boolean reqPS = false;

// Sound setup
boolean ambientSoundPlaying = false;
long soundTimer = millis();
int soundInterval = 12000; // Play a new sound every twelve seconds
boolean ambientSound = true;
int numSongs = 5; // Number of songs played in ambient sound (001 to 0XX)
boolean highVolume = false; // Denotes if driving sound is in high volume yet
boolean fastPlaying = false; // Denotes if fast-moving sound is playing
boolean drivingSoundPlaying = false;

// **************************************************
// **************************************************
//
//                      QUIZ 3
//
// **************************************************
// **************************************************
bool requestSongStart = false;
bool requestSongStop = false;
bool requestScrollUp = false;
bool requestScrollDown = false;
bool requestVolUp = false;
bool requestVolDown = false;

int currentSelectedSongNumber = 0;
int currentSelectedSongLength = 166;
String currentSelectedSongTitle = "Walk the Line";
long currentSelectedSongMillisStart = millis();
float currentSelectedSongPercentComplete = 0;
bool currentSelectedSongAtEnd = false;
bool currentSelectedSongPlaying = false;

int currentVolumeNumber = 50;  // from 0 HIGH to 100 LOW
float currentVolumePercentage = .5;

bool onMainMenu = true;
bool initMainScreenComplete = false;
bool onSongDetailScreen = false;
bool initSongDetailComplete = false;
bool onVolDetailScreen = false;
bool initVolumeScreenComplete = false;
int currentTopScrollSongNumber = 1;

long scrollScreenDelayMillis = millis();
int scrollScreenDelayInterval = 500;
long refreshPercentCompleteMillis = millis();
int refreshPercentCompleteInterval = 500;
long refreshPercentVolumeMillis = millis();
int refreshPercentVolumeInterval = 50;
int percentCompleted = 0;
int curPercentCompleted = 0;

String songTitle[36] = {"Walk the Line",
                    "Ring of Fire",
                    "Blue Suede Shoes",
                    "So Lonesome",
                    "Folsom Prison",
                    "Cheatin Heart",
                    "Jolene",
                    "Big River",
                    "Blues Eyes Cryin",
                    "Imagine",
                    "Long Tall Sally",
                    "Pretty Woman",
                    "Peggy Sue",
                    "Everyday",
                    "La Bamba",
                    "Sweet Dreams",
                    "Desperado",
                    "The Twist",
                    "Respect",
                    "People Get Ready",
                    "Dock of the Bay",
                    "Dancing Streets",
                    "My Imagination",
                    "Stay Together",
                    "Papa New Bag",
                    "Stany By Me",
                    "Who Do You Love",
                    "My Generation",
                    "Yesterday",
                    "Mr Tambourine",
                    "Fighting Man",
                    "Paranoid",
                    "Highway to Hell",
                    "Roxanne",
                    "Lola",
                    "Love Rock N Roll"};
                    
int songTrack[36]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};

long songLength[36]={166,157,136,169,165,163,162,168,140,184,129,180,150,128,124,211,121,156,148,159,162,158,181,198,128,178,150,199,127,150,139,173,208,195,250,175};


// ---------------------------------------------------------------------------------------
//    Used for Pin 13 Main Loop Blinker
// ---------------------------------------------------------------------------------------
long blinkMillis = millis();
boolean blinkOn = false;

// =======================================================================================
//                                 Main Program
// =======================================================================================
// =======================================================================================
//                                Setup Function
// =======================================================================================
void setup() {
  //Initialize Serial @ 115200 baud rate for Serial Monitor Debugging
  Serial.begin(115200);
  while (!Serial);

  //Initialize the USB Dongle and check for errors
  if (Usb.Init() == -1)
  {
    Serial.println("OSC did not start");
    while (1); //halt
  }

  Serial.println("Bluetooth Library Started");

  //PS3 Controller - sets the interrupt function to call when PS3 controller tries to connect
  PS3Controller->attachOnInit(onInitPS3Controller);

  //Setup PIN 13 for Arduino Main Loop Blinker Routine
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // ----------------------------------------------
  // YOUR SETUP CONTROL CODE SHOULD START HERE
  // ----------------------------------------------

  MP3Trigger.setup(&Serial2);
  Serial2.begin(MP3Trigger::serialRate());
  //Serial.println(MP3Trigger.getVolume());

  // OLED Display Setup
  display.begin(SSD1306_SWITCHCAPVCC, 0X3C);
  display.display(); //first display.display() shows a splash screen
  delay(2000); //delay is OK because it is in SETUP
  display.setTextSize(1); //set font to smallest size
  display.setTextColor(WHITE); //set font color
  display.clearDisplay(); //clear the current buffer
  display.println(">> " + songTitle[currentSelectedSongNumber]);
  display.println(songTitle[currentSelectedSongNumber + 1]);
  display.println(songTitle[currentSelectedSongNumber + 2]);
  display.println(songTitle[currentSelectedSongNumber + 3]);
  display.println(songTitle[currentSelectedSongNumber + 4]);
  display.display(); //send initial scroll to display


  // ----------------------------------------------
  // YOUR SETUP CONTROL CODE SHOULD END HERE
  // ---------------------------------------------
}

// =======================================================================================
//    Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================
void loop() {
  // Make sure the PS3 Controller is working - skip main loop if not
  if ( !readUSB() )
  {
    return;
  }

  // If the PS3 controller has been connected - start processing the main controller routines
  if (PS3Controller->PS3Connected) {

    // Read the PS3 Controller and set request state variables for this loop
    readPS3Request();

    // ----------------------------------------------
    // YOUR MAIN LOOP CONTROL CODE SHOULD START HERE
    // ----------------------------------------------
    
    MP3Trigger.update();

    if (onVolDetailScreen) {
      if (currentSelectedSongPlaying) {
        if (reqRightJoyUp || reqRightJoyDown) {
          changeVolume();
        }

        if ((millis() - refreshPercentVolumeMillis) > 1000)
          onVolDetailScreen = false;
      }
      
    } else {
      if (!currentSelectedSongPlaying) {
        scrollOLED();
      }

      if (currentSelectedSongPlaying) {
        continueSong();
      }

      if (reqStart && !currentSelectedSongPlaying) {
        requestSongStart = true;
        currentSelectedSongMillisStart = millis();
        Serial.println("Starting song");
        startSong();
      }
  
      if (reqSelect) {
        stopSong();
      }

      if (reqRightJoyUp || reqRightJoyDown) {
        onVolDetailScreen = true;
      }
    }

    // ----------------------------------------------
    // YOUR MAIN LOOP CONTROL CODE SHOULD END HERE
    // ----------------------------------------------

    // Ignore extra inputs from the PS3 Controller for 1/2 second from prior input
    if (extraRequestInputs) {
      if ((previousRequestMillis + 500) < millis())
      {
        extraRequestInputs = false;
      }
    }

    // If there was a PS3 request this loop - reset the request variables for next loop
    if (reqMade) {
      resetRequestVariables();
      reqMade = false;
    }
  }

  // Blink to show working heart beat on the Arduino control board
  // If Arduino LED is not blinking - the sketch has crashed
  if ((blinkMillis + 500) < millis()) {
    if (blinkOn) {
      digitalWrite(13, LOW);
      blinkOn = false;
    } else {
      digitalWrite(13, HIGH);
      blinkOn = true;
    }
    blinkMillis = millis();
  }



}

// =======================================================================================
//      ADD YOUR CUSTOM DROID FUNCTIONS STARTING HERE
// =======================================================================================

void printScrollOLED() {
  display.setCursor(0,0); //set cursor to TOP LEFT of display
  display.clearDisplay();
  if (currentSelectedSongNumber < 31) {
    display.println(">> " + songTitle[currentSelectedSongNumber]);
    display.println(songTitle[currentSelectedSongNumber + 1]);
    display.println(songTitle[currentSelectedSongNumber + 2]);
    display.println(songTitle[currentSelectedSongNumber + 3]);
    display.println(songTitle[currentSelectedSongNumber + 4]);
  }
  else {
    display.println((currentSelectedSongNumber == 31) ? ">> " + songTitle[31] : songTitle[31]);
    display.println((currentSelectedSongNumber == 32) ? ">> " + songTitle[32] : songTitle[32]);
    display.println((currentSelectedSongNumber == 33) ? ">> " + songTitle[33] : songTitle[33]);
    display.println((currentSelectedSongNumber == 34) ? ">> " + songTitle[34] : songTitle[34]);
    display.println((currentSelectedSongNumber == 35) ? ">> " + songTitle[35] : songTitle[35]);
  }

  display.display();
}

void scrollOLED() {
  if (millis() - scrollScreenDelayMillis > 200) {
    if (reqLeftJoyUp && currentSelectedSongNumber > 0) {
      scrollScreenDelayMillis = millis();
      currentSelectedSongNumber--;
      Serial.println("Scrolling Up");
      printScrollOLED();
    } else if (reqLeftJoyDown && currentSelectedSongNumber < 35) {
      scrollScreenDelayMillis = millis();
      currentSelectedSongNumber++;
      Serial.println("Scrolling Down");
      printScrollOLED();
    }
  }
  currentSelectedSongLength = songLength[currentSelectedSongNumber];
  
}

void startSong() {
  MP3Trigger.trigger(currentSelectedSongNumber + 1);
  currentSelectedSongPlaying = true; 
}

void continueSong() {
  curPercentCompleted = (millis() - currentSelectedSongMillisStart) / songLength[currentSelectedSongNumber] / 10;
  if (curPercentCompleted >= 100) {
    stopSong();
  }
  else if (curPercentCompleted != percentCompleted || requestSongStart) {
    requestSongStart = false;
    percentCompleted = curPercentCompleted;
    songDisplay();
  }
}

void stopSong() {
  currentSelectedSongPlaying = false;
  MP3Trigger.stop();
  printScrollOLED();
}

void songDisplay() {
  display.clearDisplay();
  display.setCursor(0,0); //set cursor to TOP LEFT of display
  display.println("Song Playing: ");
  display.println(songTitle[currentSelectedSongNumber]);
  display.println("% Complete: " + String(percentCompleted) + "%");
  display.display();
}

void changeVolume() {
  if ((millis() - refreshPercentVolumeMillis) > refreshPercentVolumeInterval) {
    if (reqRightJoyUp && currentVolumeNumber > 0) {
      currentVolumeNumber -= 1;
      MP3Trigger.setVolume(currentVolumeNumber);
      refreshPercentVolumeMillis = millis();
      volumeDisplay();
    }
    else if (reqRightJoyDown && currentVolumeNumber < 100) {
      currentVolumeNumber += 1;
      MP3Trigger.setVolume(currentVolumeNumber);
      refreshPercentVolumeMillis = millis();
      volumeDisplay();
    }
  }
}

void volumeDisplay() {
  display.setCursor(0,0); //set cursor to TOP LEFT of display
  display.clearDisplay();
  display.println("Volume " + String(100 - currentVolumeNumber) + "%");
  display.display();
}


// =======================================================================================
//      YOUR CUSTOM DROID FUNCTIONS SHOULD END HERE
// =======================================================================================

// =======================================================================================
//      CORE DROID CONTROL FUNCTIONS START HERE - EDIT WITH CAUTION
// =======================================================================================
// Read the PS3 Controller and set request state variables
void readPS3Request()
{
  if (!extraRequestInputs) {

    if (PS3Controller->getButtonPress(UP))
    {
      Serial.println("Button: UP Selected");

      reqArrowUp = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(DOWN))
    {
      Serial.println("Button: DOWN Selected");

      reqArrowDown = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(LEFT))
    {
      Serial.println("Button: LEFT Selected");

      reqArrowLeft = true;
      reqMade = true;
      
      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(RIGHT))
    {
      Serial.println("Button: RIGHT Selected");

      reqArrowRight = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(CIRCLE))
    {
      Serial.println("Button: CIRCLE Selected");

      reqCircle = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(CROSS))
    {
      Serial.println("Button: CROSS Selected");

      reqCross = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(TRIANGLE))
    {
      Serial.println("Button: TRIANGLE Selected");

      reqTriangle = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }


    if (PS3Controller->getButtonPress(SQUARE))
    {
      Serial.println("Button: SQUARE Selected");

      reqSquare = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(L1))
    {
      Serial.println("Button: LEFT 1 Selected");

      reqL1 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(L2))
    {
      Serial.println("Button: LEFT 2 Selected");

      reqL2 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(R1))
    {
      Serial.println("Button: RIGHT 1 Selected");

      reqR1 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(R2))
    {
      Serial.println("Button: RIGHT 2 Selected");

      reqR2 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(SELECT))
    {
      Serial.println("Button: SELECT Selected");

      reqSelect = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(START))
    {
      Serial.println("Button: START Selected");

      reqStart = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(PS))
    {
      Serial.println("Button: PS Selected");

      reqPS = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }
  }

  if (((abs(PS3Controller->getAnalogHat(LeftHatY) - 128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(LeftHatX) - 128) > joystickDeadZoneRange)))
  {
    reqLeftJoyUp = false;
    reqLeftJoyDown = false;
    reqLeftJoyLeft = false;
    reqLeftJoyRight = false;
    reqLeftJoyYValue = 0;
    reqLeftJoyXValue = 0;
    reqLeftJoyMade = true;

    int currentValueY = PS3Controller->getAnalogHat(LeftHatY) - 128;
    int currentValueX = PS3Controller->getAnalogHat(LeftHatX) - 128;

    char yString[5];
    itoa(currentValueY, yString, 10);

    char xString[5];
    itoa(currentValueX, xString, 10);

    /* Serial.print("LEFT Joystick Y Value: ");
    Serial.println(yString);
    Serial.print("LEFT Joystick X Value: ");
    Serial.println(xString); */

    if (currentValueY > joystickDeadZoneRange) {
      Serial.println("Left Joystick DOWN");
      reqLeftJoyDown = true;
      reqLeftJoyYValue = currentValueY;
    }

    if (currentValueY < (-1 * joystickDeadZoneRange)) {
      Serial.println("Left Joystick UP");
      reqLeftJoyUp = true;
      reqLeftJoyYValue = currentValueY;
    }

    if (currentValueX > joystickDeadZoneRange) {
      /*Serial.println("Left Joystick RIGHT"); */
      reqLeftJoyRight = true;
      reqLeftJoyXValue = currentValueX;
    }

    if (currentValueX < (-1 * joystickDeadZoneRange)) {
      /* Serial.println("Left Joystick LEFT"); */
      reqLeftJoyLeft = true;
      reqLeftJoyXValue = currentValueX;
    }
  } else {
    if (reqLeftJoyMade) {
      reqLeftJoyUp = false;
      reqLeftJoyDown = false;
      reqLeftJoyLeft = false;
      reqLeftJoyRight = false;
      reqLeftJoyYValue = 0;
      reqLeftJoyXValue = 0;
      reqLeftJoyMade = false;
    }
  }

  if (((abs(PS3Controller->getAnalogHat(RightHatY) - 128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(RightHatX) - 128) > joystickDeadZoneRange)))
  {
    reqRightJoyUp = false;
    reqRightJoyDown = false;
    reqRightJoyLeft = false;
    reqRightJoyRight = false;
    reqRightJoyYValue = 0;
    reqRightJoyXValue = 0;
    reqRightJoyMade = true;

    int currentValueY = PS3Controller->getAnalogHat(RightHatY) - 128;
    int currentValueX = PS3Controller->getAnalogHat(RightHatX) - 128;

    char yString[5];
    itoa(currentValueY, yString, 10);

    char xString[5];
    itoa(currentValueX, xString, 10);

    /* Serial.print("RIGHT Joystick Y Value: ");
    Serial.println(yString);
    Serial.print("RIGHT Joystick X Value: ");
    Serial.println(xString); */

    if (currentValueY > joystickDeadZoneRange) {
      Serial.println("Right Joystick DOWN");
      reqRightJoyDown = true;
      reqRightJoyYValue = currentValueY;
    }

    if (currentValueY < (-1 * joystickDeadZoneRange)) {
      Serial.println("Right Joystick UP");
      reqRightJoyUp = true;
      reqRightJoyYValue = currentValueY;
    }

    if (currentValueX > joystickDeadZoneRange) {
      Serial.println("Right Joystick RIGHT");
      reqRightJoyRight = true;
      reqRightJoyXValue = currentValueX;
    }

    if (currentValueX < (-1 * joystickDeadZoneRange)) {
      Serial.println("Right Joystick LEFT");
      reqRightJoyLeft = true;
      reqRightJoyXValue = currentValueX;
    }
  } else {
    if (reqRightJoyMade) {
      reqRightJoyUp = false;
      reqRightJoyDown = false;
      reqRightJoyLeft = false;
      reqRightJoyRight = false;
      reqRightJoyYValue = 0;
      reqRightJoyXValue = 0;
      reqRightJoyMade = false;
    }
  }
}

// Reset the PS3 request variables on every processing loop when needed
void resetRequestVariables()
{
  reqArrowUp = false;
  reqArrowDown = false;
  reqArrowLeft = false;
  reqArrowRight = false;
  reqCircle = false;
  reqCross = false;
  reqTriangle = false;
  reqSquare = false;
  reqL1 = false;
  reqL2 = false;
  reqR1 = false;
  reqR2 = false;
  reqSelect = false;
  reqStart = false;
  reqPS = false;
}

// Initialize the PS3 Controller Trying to Connect
void onInitPS3Controller() {
  PS3Controller->setLedOn(LED1);
  isPS3ControllerInitialized = true;
  badPS3Data = 0;

  mainControllerConnected = true;
  WaitingforReconnect = true;

  Serial.println("We have the controller connected");
  Serial.print("Dongle Address: ");
  String dongle_address = String(Btd.my_bdaddr[5], HEX) + ":" + String(Btd.my_bdaddr[4], HEX) + ":" + String(Btd.my_bdaddr[3], HEX) + ":" + String(Btd.my_bdaddr[2], HEX) + ":" + String(Btd.my_bdaddr[1], HEX) + ":" + String(Btd.my_bdaddr[0], HEX);
  Serial.println(dongle_address);
}

// Determine if we are having connection problems with the PS3 Controller
boolean criticalFaultDetect() {
  if (PS3Controller->PS3Connected) {
    currentTime = millis();
    lastMsgTime = PS3Controller->getLastMessageTime();
    msgLagTime = currentTime - lastMsgTime;

    if (WaitingforReconnect) {
      if (msgLagTime < 200) {
        WaitingforReconnect = false;
      }
      lastMsgTime = currentTime;
    }

    if ( currentTime >= lastMsgTime) {
      msgLagTime = currentTime - lastMsgTime;
    } else {
      msgLagTime = 0;
    }

    if ( msgLagTime > 5000 ) {
      Serial.println("It has been 5s since we heard from Controller");
      Serial.println("Disconnecting the controller");

      PS3Controller->disconnect();
      WaitingforReconnect = true;
      return true;
    }

    //Check PS3 Signal Data
    if (!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged)) {
      //We don't have good data from the controller.
      //Wait 15ms - try again
      delay(15);
      Usb.Task();
      lastMsgTime = PS3Controller->getLastMessageTime();

      if (!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged)) {
        badPS3Data++;
        Serial.println("**Invalid data from PS3 Controller. - Resetting Data**");
        return true;
      }
    } else if (badPS3Data > 0) {
      badPS3Data = 0;
    }

    if ( badPS3Data > 10 ) {
      Serial.println("Too much bad data coming from the PS3 Controller");
      Serial.println("Disconnecting the controller");

      PS3Controller->disconnect();
      WaitingforReconnect = true;
      return true;
    }
  }

  return false;
}

// USB Read Function - Supports Main Program Loop
boolean readUSB() {
  Usb.Task();
  //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  if (PS3Controller->PS3Connected) {
    if (criticalFaultDetect()) {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      return false;
    }

  }
  return true;
}
