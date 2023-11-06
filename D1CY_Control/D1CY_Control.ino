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

// ---------------------------------------------------------------------------
// Sonars Setup & Control
// ---------------------------------------------------------------------------

NewPing frontSonar = NewPing(48, 49);     // Trig on Pin#48, Echo on Pin#49
NewPing leftFrontSonar = NewPing(46, 47); // Trig on Pin#46, Echo on Pin#47
NewPing leftBackSonar = NewPing(44, 45);  // Trig on Pin#44, Echo on Pin#45
NewPing backSonar = NewPing(42, 43);      // Trig on Pin#42, Echo on Pin#43

boolean autoMode = false;
int sonarReadCycle = 1;
long sonarIntervalTimer = millis();
int sonarIntervalTime = 300;        // Take a sonar reading every 300ms
int currentFrontDistance = 0;       // Distance captured in CM
int currentLeftFrontDistance = 0;   // Distance captured in CM
int currentLeftBackDistance = 0;   // Distance captured in CM
int currentBackDistance = 0;        // Distance captured in CM
int tapeDistance = -1;               // Distance from wall to tape in CM

int totalTurns = 0;
boolean droidTurning = false;

long turnLeftIntervalTimer = millis();
int turnLeftIntervalTime = 1000;

// Sound setup
boolean soundPlaying = false;
long soundTimer = millis();
int soundInterval = 5000; // Play a new sound every five seconds
boolean ambientSound = true;
int numSongs = 5; // Number of songs played in ambient sound (001 to 0XX)


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

  myServo.attach(9);
  myServo.write(90);

  randomSeed(analogRead(0));

  Serial1.begin(9600);
  ST->autobaud();
  ST->setTimeout(200);
  ST->setDeadband(driveDeadBandRange);
  ST->setRamping(10);

  MP3Trigger.setup(&Serial2);
  Serial2.begin(MP3Trigger::serialRate());

  // OLED Display Setup
  display.begin(SSD1306_SWITCHCAPVCC, 0X3C);
  display.display(); //first display.display() shows a splash screen
  delay(2000); //delay is OK because it is in SETUP
  display.setTextSize(1); //set font to smallest size
  display.setTextColor(WHITE); //set font color
  display.clearDisplay(); //clear the current buffer
  display.display(); //send clear buffer to display


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
    
    if (reqRightJoyLeft || reqRightJoyRight) {
      moveServo();
    }

    if (PS3Controller->PS3Connected) {
      readPS3Request();
      if (reqMade) {
        // Toggle autoMode on/off using CROSS button on PS3 Controller
        if (reqCross) {
          if (autoMode) {
            autoMode = false;
          } else {
            autoMode = true;
            sonarIntervalTimer = millis();
          }
        }
      }
    }

    // If autoMode is true - start taking Sonar Readings
    if (autoMode) {
      initTapeDistance();
      
      takeSonarReadings();
      
      autoMoveDroid();
    } else {
      moveDroid();
    }

    if (reqArrowLeft) {
      turnLeft();
    }

    // Sound control
    if (reqR2) {
      ambientSound = !ambientSound;
    }
    
    MP3Trigger.update();
    if (ambientSound)
      playAmbientSound();

    // Oled control
    if (reqL2) {
      Serial.print("Printing OLED");
      printOLED();
      reqL2 = false;
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

void moveServo() {
  int curLoc = myServo.read();

  if (!((curLoc >= 180 && reqRightJoyRight) || curLoc <= 0 && reqRightJoyLeft)) {
    if (abs(reqRightJoyXValue) < 100) {
      myServo.write(reqRightJoyXValue > 0 ? (curLoc + 1) : (curLoc - 1));
    }
    else {
      myServo.write(reqRightJoyXValue > 0 ? (curLoc + 2) : (curLoc - 2));
    }

    Serial.print("Difference: ");
    Serial.println(curLoc - myServo.read());
  }
}

void moveDroid() {
  if (reqLeftJoyMade) {
    currentSpeed = reqLeftJoyYValue;
    currentTurn = reqLeftJoyXValue;
    ST->turn(currentTurn/2);
    ST->drive(currentSpeed/1.5);

    if (!droidMoving) {
      droidMoving = true;
    }
  }
  else {
    if (droidMoving) {
      ST->stop();
      droidMoving = false;
      currentTurn = 0;
      currentSpeed = 0;
    }
  }
}

void turnLeft() {
  Serial.println(turnLeftIntervalTimer);
  if ((turnLeftIntervalTimer + turnLeftIntervalTime) > millis()) {
    return;
  } else {
    turnLeftIntervalTimer = millis();
  }
  ST->drive(100);
  ST->turn(60);
  Serial.println("Turning left:");
}

void initTapeDistance() {
  if (tapeDistance == -1) {
    tapeDistance = (leftFrontSonar.convert_cm(leftFrontSonar.ping_median(5)) + leftBackSonar.convert_cm(leftBackSonar.ping_median(5))) / 2;
    Serial.println("***********INIT TAPE DISTANCE*************");
    Serial.print("Init Tape Distance: ");
    Serial.println(tapeDistance);
  }
}

void takeSonarReadings() {
  if (totalTurns < 4) {
    takeForwardSonarReadings();
  } else {
    takeBackwardSonarReadings();
  }
}

void takeForwardSonarReadings() {  
  if ((sonarIntervalTimer + sonarIntervalTime) > millis()) {
    return;
  } else {
    sonarIntervalTimer = millis();
  }

  if (sonarReadCycle == 1) {
    currentFrontDistance = frontSonar.convert_cm(frontSonar.ping_median(5));
    //Serial.println("***********FRONT SONAR*************");
    //Serial.print("Front Sonar: "); 
    //Serial.println(currentFrontDistance);
  } 
  
  if (sonarReadCycle == 2) {
    currentLeftFrontDistance = leftFrontSonar.convert_cm(leftFrontSonar.ping_median(5));
    //Serial.println("***********LEFT FRONT SONAR*************");
    //Serial.print("Left Front Sonar: ");
    //Serial.println(currentLeftFrontDistance);
  }

    if (sonarReadCycle == 3) {
    currentFrontDistance = frontSonar.convert_cm(frontSonar.ping_median(5));
    //Serial.println("***********FRONT SONAR*************");
    //Serial.print("Front Sonar: "); 
    //Serial.println(currentFrontDistance);
  } 

  if (sonarReadCycle == 4) {
    currentLeftBackDistance = leftBackSonar.convert_cm(leftBackSonar.ping_median(5));
    //Serial.println("***********LEFT BACK SONAR*************");
    //Serial.print("Left Back Sonar: ");
    //Serial.println(currentLeftBackDistance);
  }

  sonarReadCycle++;

  if (sonarReadCycle == 5) {
    sonarReadCycle = 1;
  }
}

void takeBackwardSonarReadings() {
  if ((sonarIntervalTimer + sonarIntervalTime) > millis()) {
    return;
  } else {
    sonarIntervalTimer = millis();
  }

  if (sonarReadCycle == 1) {
    currentBackDistance = backSonar.convert_cm(backSonar.ping_median(5));
    //Serial.println("***********BACK SONAR*************");
    //Serial.print("Back Sonar: ");
    //Serial.println(currentBackDistance);
  }

  
  if (sonarReadCycle == 2) {
    currentLeftFrontDistance = leftFrontSonar.convert_cm(leftFrontSonar.ping_median(5));
    //Serial.println("***********LEFT FRONT SONAR*************");
    //Serial.print("Left Front Sonar: ");
    //Serial.println(currentLeftFrontDistance);
  }

  if (sonarReadCycle == 3) {
    currentBackDistance = backSonar.convert_cm(backSonar.ping_median(5));
    //Serial.println("***********BACK SONAR*************");
    //Serial.print("Back Sonar: ");
    //Serial.println(currentBackDistance);
  }


  if (sonarReadCycle == 4) {
    currentLeftBackDistance = leftBackSonar.convert_cm(leftBackSonar.ping_median(5));
    //Serial.println("***********LEFT BACK SONAR*************");
    //Serial.print("Left Back Sonar: ");
    //Serial.println(currentLeftBackDistance);
  }

  sonarReadCycle++;

  if (sonarReadCycle == 5) {
    sonarReadCycle = 1;
  }
}

void autoMoveDroid() {
  int driveSpeed = -40;
  if (currentFrontDistance > (tapeDistance - 15)) {
    if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 10) {
      ST->drive(driveSpeed);
      ST->turn(15);
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 5) {
      ST->drive(driveSpeed);
      ST->turn(10);
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 2) {
      ST->drive(driveSpeed);
      ST->turn(5);
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 10) {
      ST->drive(driveSpeed);
      ST->turn(-12);
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 5) {
      ST->drive(driveSpeed);
      ST->turn(-8);
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 2) {
      ST->drive(driveSpeed);
      ST->turn(-4);
    } else {
      ST->drive(driveSpeed);
      ST->turn(0);
      Serial.println("***********MOVING FORWARD*************");
    }
  } else {   
    ST->drive(15);
    ST->turn(48);
    Serial.println("***********TURNING RIGHT*************");
  }
  
  
  if (!droidMoving) {
    droidMoving = true;
  }
}

void printOLED() {
  display.setCursor(0,0); //set cursor to TOP LEFT of display
  display.println("DIR: NE");
  display.println(" ");
  display.println("D: 20.2 Ft");
  display.println(" ");
  display.display();
}

void playAmbientSound() {
  if (soundPlaying == false || millis() > (soundTimer + soundInterval)) {
    MP3Trigger.trigger(random(1,numSongs + 1));
    soundPlaying = true;
    soundTimer = millis();
  }
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

      turnLeftIntervalTimer = millis();
      Serial.println(reqArrowLeft);

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

      autoMode = !autoMode;
      tapeDistance = -1;

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

    Serial.print("LEFT Joystick Y Value: ");
    Serial.println(yString);
    Serial.print("LEFT Joystick X Value: ");
    Serial.println(xString);

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

    Serial.print("RIGHT Joystick Y Value: ");
    Serial.println(yString);
    Serial.print("RIGHT Joystick X Value: ");
    Serial.println(xString);

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
