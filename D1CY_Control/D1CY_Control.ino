// =======================================================================================
//                            D1CY_Control program for D1CY
// =======================================================================================
//                            Last Revised Date: 12/09/2023
//                       Written By: Thomas Mercurio and Jon Nguyen
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
long drivingStart; // Stores when the droid started driving
long drivingStop; // Store when the droid stopped driving
long autoRightTurnTimer = millis();
long autoLeftTurnTimer = millis();

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

// ---------------------------------------------------------------------------
// Driving Setup
// ---------------------------------------------------------------------------

boolean autoMode = false;
int sonarReadCycle = 1;
long sonarIntervalTimer = millis();
int sonarIntervalTime = 300;        // Take a sonar reading every 300ms
int currentFrontDistance = 0;       // Distance captured in CM
int currentLeftFrontDistance = 0;   // Distance captured in CM
int currentLeftBackDistance = 0;   // Distance captured in CM
int currentBackDistance = 0;        // Distance captured in CM
int tapeDistance = -1;               // Distance from wall to tape in CM

int driveSpeed = -40;
int turnSpeed = 0;

int totalTurns = 0;
boolean droidTurning = false;

float turnIncrease = 1.0;
float speedIncrease = 1.0;

float r2speedBoost = 1.0;
float r2turnBoost = 1.6;

long turnLeftIntervalTimer = millis();
int turnLeftIntervalTime = 1000;

// -------------------------------------------------------------------------------------
// Sound Setup
// -------------------------------------------------------------------------------------

MP3Trigger MP3Trigger;
boolean ambientSoundPlaying = false;
long soundTimer = millis();
int soundInterval = 12000; // Play a new sound every twelve seconds
boolean ambientSound = true;
int numSongs = 5; // Number of songs played in ambient sound (001 to 0XX)
boolean highVolume = false; // Denotes if driving sound is in high volume yet
boolean fastPlaying = false; // Denotes if fast-moving sound is playing
boolean drivingSoundPlaying = false;
float routine3mspb = 454.54; // Cotton Eye Joe ms per beat (132 bpm)

#define BASE_VOLUME 50


// ---------------------------------------------------------------------------------------
// LED Setup
// ---------------------------------------------------------------------------------------

#define clock 5
#define data 4
#define latch 6

Adafruit_TLC5947 LEDControl = Adafruit_TLC5947(1, clock, data, latch);

int ledMaxBright = 4000;   // 4095 is MAX brightness
long lightStart = millis();
bool lightsStarted = false;
bool ambientLighting = true;

bool switchMade = false;
int curLight = 20;
int nextLight = 0;

bool doorOpen = false;

int curLightingLoop = 0;

// ---------------------------------------------------------------------------------------
// Integrated Routine Setup
// ---------------------------------------------------------------------------------------
int routineNumber = 0;
long routineStart;
int curSubRoutine = 0;
long subRoutineStart;
bool routineSongPlaying = false;
long arduinoStart = millis();

// Used for routine1
bool redLightsOn = false;
bool blueLightsOn = false;
bool greenLightsOn = false;
bool yellowLightsOn = false;

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
  myServo.write(115);

  randomSeed(analogRead(0));

  Serial1.begin(9600);
  ST->autobaud();
  ST->setTimeout(200);
  ST->setDeadband(driveDeadBandRange);
  ST->setRamping(10);

  MP3Trigger.setup(&Serial2);
  Serial2.begin(MP3Trigger::serialRate());
  MP3Trigger.setVolume(BASE_VOLUME);

  // OLED Display Setup
  display.begin(SSD1306_SWITCHCAPVCC, 0X3C);
  display.display(); //first display.display() shows a splash screen
  delay(2000); //delay is OK because it is in SETUP
  display.setTextSize(1); //set font to smallest size
  display.setTextColor(WHITE); //set font color
  display.clearDisplay(); //clear the current buffer
  display.display(); //send clear buffer to display


  LEDControl.begin();
  clearLights();
  
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

    MP3Trigger.update();

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
      // Reset routine
      else if (reqSelect) {
        if (routineNumber != 0) {
          routineNumber = 0;
          curSubRoutine = 0;
          clearLights();
          routineSongPlaying = false;
          MP3Trigger.stop();
          ST->drive(0);
          ST->turn(0);
        }
      }
      // Start integrated routine 1
      else if (reqCircle && routineNumber == 0) {
        ambientSound = false;
        Serial.println("routine1 starting");
        routineNumber = 1;
        routineStart = millis();
      }
      else if (reqTriangle && routineNumber == 0) {
        ambientSound = false;
        Serial.println("routine2 starting");
        routineNumber = 2;
        routineStart = millis();
      }
      else if (reqSquare && routineNumber == 0) {
        Serial.println("routine3 starting");
        routineNumber = 3;
        routineStart = millis();
      }
      else if (reqR2) {
        if (ambientSound) {
          MP3Trigger.stop();
        }
        MP3Trigger.setVolume(BASE_VOLUME);
        ambientSound = !ambientSound;
      }
      else if (reqR1) {
        if (ambientLighting) {
          clearLights();
        }
        ambientLighting = !ambientLighting;
      }
    }

    // If autoMode is true - start taking Sonar Readings
    if (autoMode) {
      initTapeDistance();
      
      takeSonarReadings();

      if (totalTurns < 4) {
        autoMoveDroidForward();
      } else {
        autoMoveDroidBackward();
      }
      
    } else if (routineNumber == 1) {
      routine1();
    }
    
    else if (routineNumber == 2) {
      routine2();
    }

    else if (routineNumber == 3) {
      routine3();
    }

    else {
      moveDroid();
      
      if (ambientSound)
        playAmbientSound();
  
      if (ambientLighting) 
        displayLighting();
  
      // Oled control
      /*
      if (reqL2) {
        Serial.print("Printing OLED");
        printOLED();
        reqL2 = false;
      }
      */
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

  if (!((curLoc >= 180 && reqRightJoyRight) || curLoc <= 115 && reqRightJoyLeft)) {
    if (abs(reqRightJoyXValue) < 100) {
      myServo.write(reqRightJoyXValue > 0 ? (curLoc + 1) : (curLoc - 1));
    }
    else {
      myServo.write(reqRightJoyXValue > 0 ? (curLoc + 2) : (curLoc - 2));
    }

    Serial.print("Angle: ");
    Serial.println(myServo.read());
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
      drivingStart = millis();
      ambientSound = false;
    }
    //Serial.println("Called playDrivingSound");
    //playDrivingSound(true);
  }
  else {
    if (droidMoving) {
      ST->stop();
      droidMoving = false;
      currentTurn = 0;
      currentSpeed = 0;
      //playDrivingSound(false);
      drivingStop = millis();
    }
  }
}

// Takes 800 milliseconds to turn left
void turnLeft() {
  //Serial.println(turnLeftIntervalTimer);
  ST->turn(-90 * turnIncrease);
  ST->drive(-5);
  //Serial.println("Turning left:");
}

// Takes 800 milliseconds to turn right
void turnRight() {
  //Serial.println(turnRightIntervalTimer);
  ST->turn(90 * turnIncrease);
  ST->drive(-5);
  //Serial.println("Turning right:");
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
  } 
  
  if (sonarReadCycle == 2) {
    currentLeftFrontDistance = leftFrontSonar.convert_cm(leftFrontSonar.ping_median(5));
  }

  if (sonarReadCycle == 3) {
    currentFrontDistance = frontSonar.convert_cm(frontSonar.ping_median(5));
  } 

  if (sonarReadCycle == 4) {
    currentLeftBackDistance = leftBackSonar.convert_cm(leftBackSonar.ping_median(5));
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
  }

  
  if (sonarReadCycle == 2) {
    currentLeftFrontDistance = leftFrontSonar.convert_cm(leftFrontSonar.ping_median(5));
  }

  if (sonarReadCycle == 3) {
    currentBackDistance = backSonar.convert_cm(backSonar.ping_median(5));
  }


  if (sonarReadCycle == 4) {
    currentLeftBackDistance = leftBackSonar.convert_cm(leftBackSonar.ping_median(5));
  }

  sonarReadCycle++;

  if (sonarReadCycle == 5) {
    sonarReadCycle = 1;
  }
}

void autoMoveDroidForward() {  
  if (millis() - autoRightTurnTimer < 3000) {
    driveSpeed = 10;
    turnSpeed = 30;
    Serial.println("***********TURNING RIGHT*************");
  } else if (currentFrontDistance > (tapeDistance - 15)) {
    droidTurning = false;
    driveSpeed = -30;
    
    if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 10) {
      turnSpeed = 15;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 5) {
      turnSpeed = 10;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 2) {
      turnSpeed = 5;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 10) {
      turnSpeed = -12;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 5) {
      turnSpeed = -8;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 2) {
      turnSpeed = -4;
    } else {
      turnSpeed = 0;
      Serial.println("***********MOVING FORWARD*************");
    }
  } else {   
    if (!droidTurning) {
      droidTurning = true;
      autoRightTurnTimer = millis();
      totalTurns++;
    }
  }

  ST->drive(driveSpeed);
  ST->turn(turnSpeed);
  
  if (!droidMoving) {
    droidMoving = true;
  }
}

void autoMoveDroidBackward() {  
  if (droidTurning && millis() - autoLeftTurnTimer < 3000) {
    driveSpeed = -10;
    turnSpeed = 30;
    Serial.println("***********TURNING RIGHT*************");
  } else if (currentBackDistance > (tapeDistance - 15)) {
    droidTurning = false;
    driveSpeed = 30;
    
    if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 10) {
      turnSpeed = -15;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 5) {
      turnSpeed = -10;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 < tapeDistance - 2) {
      turnSpeed = -5;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 10) {
      turnSpeed = 12;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 5) {
      turnSpeed = 8;
    } else if ((currentLeftFrontDistance + currentLeftBackDistance)/2 > tapeDistance + 2) {
      turnSpeed = 4;
    } else {
      turnSpeed = 0;
      Serial.println("***********MOVING FORWARD*************");
    }
  } else {   
    if (!droidTurning) {
      droidTurning = true;
      autoRightTurnTimer = millis();
      totalTurns++;
    }
  }

  ST->drive(driveSpeed);
  ST->turn(turnSpeed);
  
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
  /*
  if (droidMoving) {
    if (ambientSoundPlaying) {
      Serial.println("Ambient sound stopping");
      MP3Trigger.stop();
      ambientSoundPlaying = false;
    }
  }
  */
  if ((ambientSoundPlaying == false || millis() > (soundTimer + soundInterval))) {
    MP3Trigger.trigger(random(1,numSongs + 1));
    ambientSoundPlaying = true;
    soundTimer = millis();
    curLightingLoop = random(1, numSongs + 1);
    lightStart = millis();
  }
}


void displayLighting() {
  long curTime = millis() - lightStart;
  /*
  switch (curLightingLoop) {
    case 1:
      lighting1();
      break;
    case 2:
      lighting2();
      break;
    case 3:
      lighting3();
      break;
    case 4:
      lighting4();
      break;
    case 5:
      lighting5();
      break;
    LEDControl.setPWM(13, ledMaxBright / 4);
    LEDControl.setPWM(12, 0);
    LEDControl.setPWM(14, ledMaxBright / 2);
    LEDControl.setPWM(15, ledMaxBright / 4);
    LEDControl.setPWM(16, ledMaxBright / 4);
    LEDControl.setPWM(17, ledMaxBright / 4);
    LEDControl.write();
    lightsStarted = true;
    Serial.println("Lights starting");
  }
  */

  if (LEDControl.getPWM(12) > 0 && (millis() - lightStart) > 20000) {
    LEDControl.setPWM(12, 0);
    LEDControl.write();
  }
  else if ((LEDControl.getPWM(12) == 0) && ((millis() - lightStart) < 20000)) {
    LEDControl.setPWM(12, ledMaxBright / 2);
    LEDControl.write();
  }
 
}

void lighting1() {
  long curTime = millis() - lightStart;
/*
  if (curTime < (soundInterval / 2)) {
    if (curTime % 2400 < 1200) {
      if (
    }
  }
  */
}

/*
// Parameter driving denotes if driving forward or stopping
void playDrivingSound(boolean driving) {
  //Serial.println("In playDrivingSound");
  if (driving) {
    int time_driving = millis() - drivingStart;
    Serial.println(time_driving);
    if (time_driving < 1000 && !drivingSoundPlaying) {
      Serial.println("Starting Thunderstruck");
      MP3Trigger.trigger(21); // First Thunderstruck sound
      highVolume = false;
      // soundPlaying = true;
      drivingSoundPlaying = true;
    }
    else if (time_driving < 2000 && !highVolume) {
      Serial.println("Thunderstruck louder");
      MP3Trigger.setVolume(10);
      highVolume = true;
    }
    else if (!fastPlaying) {
      Serial.println("Thunderstruck loudest");
      MP3Trigger.stop();
      MP3Trigger.trigger(22);
      fastPlaying = true;
    }
  }
  else {
    Serial.println("Playing stopping sound");
    MP3Trigger.stop();
    MP3Trigger.trigger(23);
    drivingStop = millis();
    highVolume = false;
    fastPlaying = false;
    drivingSoundPlaying = false;
  }
}
*/



// Draw D1CY and play songs corresponding to the starting letters, with different 
// colored lights for different songs
void routine1() {
  if ((millis() - routineStart) < 13000) {
    drawD();
  }
  else if ((millis() - routineStart) < 32000) {
    draw1();
  }
  else if ((millis() - routineStart) < 46000) {
    drawC();
  }
  else if ((millis() - routineStart) < 59000) {
    drawY();
  }
  else if ((millis() - routineStart) > 65000) {
    Serial.println(millis());
    Serial.println(routineStart);
    Serial.println("Setting routineNumber back to 0");
    routineNumber = 0;
    routineSongPlaying = false;
    clearLights();
    MP3Trigger.setVolume(BASE_VOLUME);
  }
}

// Starts facing forward
void drawD() {
  if (!routineSongPlaying && (millis() - routineStart) < 1000) {    
    clearLights();
    
    // Turn red lights on
    LEDControl.setPWM(0, ledMaxBright);
    LEDControl.setPWM(4, ledMaxBright);
    LEDControl.setPWM(8, ledMaxBright);
    LEDControl.setPWM(12, ledMaxBright);
    LEDControl.setPWM(16, ledMaxBright);
    LEDControl.setPWM(20, ledMaxBright);
    LEDControl.write();

    //Serial.println("Song playing");
    MP3Trigger.trigger(41); // D song
    routineSongPlaying = true;
  }
  
  if ((millis() - routineStart) < 4000) {
    ST->drive(-48);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 5000) {
    turnRight();
  }
  else if ((millis() - routineStart) < 10800) {
    ST->drive(-50);
    ST->turn(25 * turnIncrease);
  }
  else if ((millis() - routineStart) < 12120) {
    turnRight();
  }
  else {
    ST->drive(0);
    ST->turn(0);
  }
  
  if ((millis() - routineStart) > 11000) {
    if (routineSongPlaying) {
      MP3Trigger.stop();
      routineSongPlaying = false;

      // Turn red lights off
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(4, 0);
      LEDControl.setPWM(8, 0);
      LEDControl.setPWM(12, 0);
      LEDControl.setPWM(16, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.write();
    }
  }
}

// Starts when routineStart = 13000 ms -- starts in bottom left facing forward
void draw1() {
  if (!routineSongPlaying && (millis() - routineStart) < 14000) {
    //Serial.println("Song playing");
    MP3Trigger.trigger(42); // 1 song -- What Makes You Beautiful
    routineSongPlaying = true;
    
    // Turn on blue lights
    LEDControl.setPWM(3, ledMaxBright);
    LEDControl.setPWM(7, ledMaxBright);
    LEDControl.setPWM(11, ledMaxBright);
    LEDControl.setPWM(15, ledMaxBright);
    LEDControl.setPWM(19, ledMaxBright);
    LEDControl.setPWM(23, ledMaxBright);
    LEDControl.write();
  }
  
  if ((millis() - routineStart) < 13900) {
    turnRight();
  }
  else if ((millis() - routineStart) < 17700) { // Bottom of 1
    ST->drive(-48);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 19700) { // Move to middle bottom of 1
    ST->drive(48);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 20440) { // Turn left
    turnLeft();
  }
  else if ((millis() - routineStart) < 23200) { // Draw middle line of 1
    ST->drive(-48);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 24500) { // Turn 135°
    turnLeft();
  }
  else if ((millis() - routineStart) < 25000) { // Draw top tip of 1
    ST->drive(-48);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 29700) { // Return to starting position
    ST->drive(-60);
    ST->turn(-30 * turnIncrease);
  }
  else if ((millis() - routineStart) < 31300) { // Rotate to face left
    turnLeft();
  }
  else {
    ST->drive(0);
    ST->turn(0);
  }
  
  // Turn lights and song off at end of number
  if ((millis() - routineStart) > 26200) {
    if (routineSongPlaying) {
      MP3Trigger.stop();
      routineSongPlaying = false;
      
      // Turn off blue lights
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(7, 0);
      LEDControl.setPWM(11, 0);
      LEDControl.setPWM(15, 0);
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.write();
    }
  }

}

// Starts when millis() - routineStart = 32000 -- starts in bottom right facing left
void drawC() {
  if (!routineSongPlaying && (millis() - routineStart) < 33000) {
    Serial.println("Song playing");
    MP3Trigger.trigger(43); // C song -- Come & Go
    routineSongPlaying = true;
    // Turn on green lights
    LEDControl.setPWM(2, ledMaxBright);
    LEDControl.setPWM(6, ledMaxBright);
    LEDControl.setPWM(10, ledMaxBright);
    LEDControl.setPWM(14, ledMaxBright);
    LEDControl.setPWM(18, ledMaxBright);
    LEDControl.setPWM(22, ledMaxBright);
    LEDControl.write();
  }

  if ((millis() - routineStart) < 38700) { // Draw C
    ST->drive(-55);
    ST->turn(24 * turnIncrease);
  }
  else if ((millis() - routineStart) < 39700) { // Stop at end of C
    ST->drive(0);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 40700) { // Turn around
    turnRight();
  }
  else if ((millis() - routineStart) < 44000) { // Drive to bottom of Y
    ST->drive(-80);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 45600) { // Turn around
    turnRight();
  }
  else {
    ST->drive(0);
    ST->turn(0);
  }
  
  if ((millis() - routineStart) > 40000) {
    if (routineSongPlaying) {
      MP3Trigger.stop();
      routineSongPlaying = false;
      
      // Turn off green lights
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(6, 0);
      LEDControl.setPWM(10, 0);
      LEDControl.setPWM(14, 0);
      LEDControl.setPWM(18, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.write();
    }
  }

}

// Starts when millis() - routineStart > 46000 -- starts in bottom middle facing forward
void drawY() {
  if (!routineSongPlaying && (millis() - routineStart) < 47000) {
    //Serial.println("Song playing");
    MP3Trigger.setVolume(20);
    MP3Trigger.trigger(44); // Y song -- You Make My Dreams Come True
    routineSongPlaying = true;
    // Turn on yellow lights
    LEDControl.setPWM(1, ledMaxBright);
    LEDControl.setPWM(5, ledMaxBright);
    LEDControl.setPWM(9, ledMaxBright);
    LEDControl.setPWM(13, ledMaxBright);
    LEDControl.setPWM(17, ledMaxBright);
    LEDControl.setPWM(21, ledMaxBright);
    LEDControl.write();
  }

  if ((millis() - routineStart) < 48000) { // Draw stem of Y
    ST->drive(-60);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 48600) { // Turn slightly left
    turnLeft();
  }
  else if ((millis() - routineStart) < 50000) { // Draw left branch of Y
    ST->drive(-60);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 51400) { // Retrace left branch of Y
    ST->drive(60);
    ST->turn(0);
  }
  else if ((millis() - routineStart) < 52100) { // Rotate right
    turnRight();
  }
  else if ((millis() - routineStart) < 53700) { // Draw right branch of Y
    ST->drive(-60);
    ST->turn(0);
  }
  else { // Stop moving
    ST->drive(0);
    ST->turn(0);
  }

  if ((millis() - routineStart) > 54000 && routineSongPlaying) {
    MP3Trigger.stop();
    routineSongPlaying = false;
    
    // Turn off yellow lights
    LEDControl.setPWM(1, 0);
    LEDControl.setPWM(5, 0);
    LEDControl.setPWM(9, 0);
    LEDControl.setPWM(13, 0);
    LEDControl.setPWM(17, 0);
    LEDControl.setPWM(21, 0);
    LEDControl.write();
  }

  if (millis() - routineStart > 55000) { // Finale of routine with lights
    if ((millis() - routineStart) < 56000) {
      if (!redLightsOn) {
        MP3Trigger.setVolume(5);
        MP3Trigger.trigger(45); // Lights flipping on sound
        // Turn on red lights
        LEDControl.setPWM(0, ledMaxBright);
        LEDControl.setPWM(4, ledMaxBright);
        LEDControl.setPWM(8, ledMaxBright);
        LEDControl.setPWM(12, ledMaxBright);
        LEDControl.setPWM(16, ledMaxBright);
        LEDControl.setPWM(20, ledMaxBright);
        LEDControl.write();
    
        redLightsOn = true;
      }
    }
  
    else if ((millis() - routineStart) < 57000) {
      if (!blueLightsOn) {
        //MP3Trigger.stop();
        MP3Trigger.trigger(45); // Lights flipping on sound
        // Turn on blue lights
        LEDControl.setPWM(0, ledMaxBright);
        LEDControl.setPWM(4, ledMaxBright);
        LEDControl.setPWM(8, ledMaxBright);
        LEDControl.setPWM(12, ledMaxBright);
        LEDControl.setPWM(16, ledMaxBright);
        LEDControl.setPWM(20, ledMaxBright);
        LEDControl.setPWM(3, ledMaxBright);
        LEDControl.setPWM(7, ledMaxBright);
        LEDControl.setPWM(11, ledMaxBright);
        LEDControl.setPWM(15, ledMaxBright);
        LEDControl.setPWM(19, ledMaxBright);
        LEDControl.setPWM(23, ledMaxBright);
        LEDControl.write();
    
        blueLightsOn = true;
      }
    }
  
    else if ((millis() - routineStart) < 58000) {
      if (!greenLightsOn) {
        //MP3Trigger.stop();
        MP3Trigger.trigger(45); // Lights flipping on sound
        // Turn on green lights
        LEDControl.setPWM(0, ledMaxBright);
        LEDControl.setPWM(4, ledMaxBright);
        LEDControl.setPWM(8, ledMaxBright);
        LEDControl.setPWM(12, ledMaxBright);
        LEDControl.setPWM(16, ledMaxBright);
        LEDControl.setPWM(20, ledMaxBright);
        LEDControl.setPWM(3, ledMaxBright);
        LEDControl.setPWM(7, ledMaxBright);
        LEDControl.setPWM(11, ledMaxBright);
        LEDControl.setPWM(15, ledMaxBright);
        LEDControl.setPWM(19, ledMaxBright);
        LEDControl.setPWM(23, ledMaxBright);
        LEDControl.setPWM(2, ledMaxBright);
        LEDControl.setPWM(6, ledMaxBright);
        LEDControl.setPWM(10, ledMaxBright);
        LEDControl.setPWM(14, ledMaxBright);
        LEDControl.setPWM(18, ledMaxBright);
        LEDControl.setPWM(22, ledMaxBright);
        LEDControl.write();
    
        greenLightsOn = true;
      }
    }
  
    else if ((millis() - routineStart) < 59000) {
      if (!yellowLightsOn) {
        //MP3Trigger.stop();
        MP3Trigger.trigger(45); // Lights flipping on sound
        // Turn on yellow lights
        LEDControl.setPWM(0, ledMaxBright);
        LEDControl.setPWM(4, ledMaxBright);
        LEDControl.setPWM(8, ledMaxBright);
        LEDControl.setPWM(12, ledMaxBright);
        LEDControl.setPWM(16, ledMaxBright);
        LEDControl.setPWM(20, ledMaxBright);
        LEDControl.setPWM(3, ledMaxBright);
        LEDControl.setPWM(7, ledMaxBright);
        LEDControl.setPWM(11, ledMaxBright);
        LEDControl.setPWM(15, ledMaxBright);
        LEDControl.setPWM(19, ledMaxBright);
        LEDControl.setPWM(23, ledMaxBright);
        LEDControl.setPWM(2, ledMaxBright);
        LEDControl.setPWM(6, ledMaxBright);
        LEDControl.setPWM(10, ledMaxBright);
        LEDControl.setPWM(14, ledMaxBright);
        LEDControl.setPWM(18, ledMaxBright);
        LEDControl.setPWM(22, ledMaxBright);
        LEDControl.setPWM(1, ledMaxBright);
        LEDControl.setPWM(5, ledMaxBright);
        LEDControl.setPWM(9, ledMaxBright);
        LEDControl.setPWM(13, ledMaxBright);
        LEDControl.setPWM(17, ledMaxBright);
        LEDControl.setPWM(21, ledMaxBright);
        LEDControl.write();
    
        yellowLightsOn = true;
      }
    }
  
    else if ((millis() - routineStart) < 60000) {
      if (!routineSongPlaying) {
        MP3Trigger.trigger(46); // Cheering noise
        routineSongPlaying = true;
      }
    }
  }

}

void routine2() {
  long curTime = millis() - routineStart;
  
  if (curTime < 17000) {
    if (curSubRoutine != 1) {
      subRoutineStart = millis();
      curSubRoutine = 1;
    }
    playTikTok();
  }
  else if (curTime < 27500) {
    if (curSubRoutine != 2) {
      subRoutineStart = millis();
      curSubRoutine = 2;
    }
    clubSetup();
  }
  else if (curTime < 40500) {
    if (curSubRoutine != 3) {
      subRoutineStart = millis();
      curSubRoutine = 3;
    }
    clubDancing();
  }
  else if (curTime < 62000) {
    if (curSubRoutine != 4) {
      subRoutineStart = millis();
      curSubRoutine = 4;
    }
    playClosingTime();
  }
  else {
    Serial.println(millis());
    Serial.println(routineStart);
    Serial.println("Setting routineNumber back to 0");
    routineNumber = 0;
    curSubRoutine = 0;
    routineSongPlaying = false;
    clearLights();
    MP3Trigger.stop();
    MP3Trigger.setVolume(BASE_VOLUME);
  }
}


void playTikTok() {
  long curTime = millis() - subRoutineStart;
  
  if (curTime < 1000) {
    if (!routineSongPlaying) {
      MP3Trigger.trigger(51); // Play Tik Tok
      clearLights();
      routineSongPlaying = true;
    }
  }
  
  if (curTime < 4000) {
    if (LEDControl.getPWM(1) == 0) {
      // Turn on front two yellow/orange lights to represent waking up
      LEDControl.setPWM(1, ledMaxBright);
      LEDControl.setPWM(21, ledMaxBright);
      LEDControl.write();
    }
  }

  else if (curTime < 4500) {
    // Turn on all front lights as glasses go on
    if (LEDControl.getPWM(0) == 0) {
      LEDControl.setPWM(0, ledMaxBright);
      LEDControl.setPWM(2, ledMaxBright);
      LEDControl.setPWM(3, ledMaxBright);
      LEDControl.setPWM(20, ledMaxBright);
      LEDControl.setPWM(22, ledMaxBright);
      LEDControl.setPWM(23, ledMaxBright);
      LEDControl.write();
    }
  }

  else if (curTime < 6000) {
    if (!doorOpen) {
      openDoor(1);
      //Serial.println("Open door");
    }
    
    ST->drive(-60 * r2speedBoost); // Drive forward to represent going out the door
    ST->turn(0);
  }
  
  else if (curTime < 7000) {
    ST->drive(0);
  }

  else if (curTime < 8500) {
    ST->drive(40 * r2speedBoost); // Drive backward for "before I leave"
    
    if (curTime < 7000) {
      if (doorOpen) {
        closeDoor(1);
        //Serial.println("Close door");
      }
    }
  }

  else if (curTime < 12000) {
    // Drive back and forth for the brush my teeth part
    if (curTime < 9000) {
      if (curTime > 8750) {
        ST->turn(-45 * r2turnBoost);
      }
    }
    
    else if (curTime < 9500) {
      ST->turn(45 * r2turnBoost);
      ST->drive(0);
      if (LEDControl.getPWM(16) == 0) {
        LEDControl.setPWM(16, ledMaxBright);
        LEDControl.setPWM(0, 0);
        LEDControl.setPWM(20, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 10000) {
      ST->turn(-45 * r2turnBoost);
      if (LEDControl.getPWM(12) == 0) {
        LEDControl.setPWM(12, ledMaxBright);
        LEDControl.setPWM(16, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 10500) {
      ST->turn(45 * r2turnBoost);
      if (LEDControl.getPWM(8) == 0) {
        LEDControl.setPWM(8, ledMaxBright);
        LEDControl.setPWM(12, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 11000) {
      ST->turn(-45 * r2turnBoost);
      if (LEDControl.getPWM(4) == 0) {
        LEDControl.setPWM(4, ledMaxBright);
        LEDControl.setPWM(8, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 11500) {
      ST->turn(45 * r2turnBoost);
      if (LEDControl.getPWM(0) == 0) {
        LEDControl.setPWM(0, ledMaxBright);
        LEDControl.setPWM(4, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 12000) {
      ST->turn(-45 * r2turnBoost);
      if (LEDControl.getPWM(20) == 0) {
        LEDControl.setPWM(20, ledMaxBright);
        LEDControl.setPWM(0, 0);
        LEDControl.write();
      }
    }
  }
  
  else if (curTime < 13000) {
    ST->drive(0);
    ST->turn(0);
  }

  else if (curTime < 14000) {
    if (LEDControl.getPWM(1) > 0) {
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(1, 0);
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.write();
    }
  }
  else if (curTime < 16000) {
    ST->drive(-40 * r2speedBoost);
    ST->turn(0);
    
    if (curTime > 15200 && routineSongPlaying) {
      MP3Trigger.stop();
      Serial.println("Stopping Tik Tok (curTime = " + String(millis() - routineStart));
      routineSongPlaying = false;
    }
  }
}

void clubSetup() {
  long curTime = millis() - subRoutineStart;

  if (curTime < 6000) {
    if (!routineSongPlaying) {
      Serial.println("Turning volume up (curTime = " + String(millis() - routineStart));
      MP3Trigger.setVolume(2);
      MP3Trigger.trigger(52); // Play setup sounds
      routineSongPlaying = true;
    }

    if (curTime < 1000) {
      turnRight();
    }
    else if (curTime % 2000 > 1000) {
      ST->drive(-40 * r2speedBoost);
      ST->turn(0);
    }
    else {
      ST->drive(40 * r2speedBoost);
      ST->turn(0);
    }
  }

  else if (curTime < 6100) {
    routineSongPlaying = false;
    MP3Trigger.stop();
    ST->drive(0);
  }

  else if (curTime < 8100) {
    if (curTime < 6500) {
      if (!routineSongPlaying) {
        MP3Trigger.trigger(53); // Lights turning on
      }  
    }

    else if (curTime < 6800) {
      if (LEDControl.getPWM(2) == 0) {
        LEDControl.setPWM(2, ledMaxBright / 4);
        LEDControl.setPWM(3, ledMaxBright / 4);
        LEDControl.setPWM(6, ledMaxBright / 4);
        LEDControl.setPWM(7, ledMaxBright / 4);
        LEDControl.setPWM(10, ledMaxBright / 4);
        LEDControl.setPWM(11, ledMaxBright / 4);
        LEDControl.write();
      }
    }
    else if (curTime < 7100) {
      if (LEDControl.getPWM(2) != 0) {
        LEDControl.setPWM(2, 0);
        LEDControl.setPWM(3, 0);
        LEDControl.setPWM(6, 0);
        LEDControl.setPWM(7, 0);
        LEDControl.setPWM(10, 0);
        LEDControl.setPWM(11, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 7400) {
      if (LEDControl.getPWM(2) == 0) {
        LEDControl.setPWM(2, ledMaxBright / 2);
        LEDControl.setPWM(3, ledMaxBright / 2);
        LEDControl.setPWM(6, ledMaxBright / 2);
        LEDControl.setPWM(7, ledMaxBright / 2);
        LEDControl.setPWM(10, ledMaxBright / 2);
        LEDControl.setPWM(11, ledMaxBright / 2);
        LEDControl.write();
      }
    }
    else if (curTime < 7700) {
      if (LEDControl.getPWM(2) != 0) {
        LEDControl.setPWM(2, 0);
        LEDControl.setPWM(3, 0);
        LEDControl.setPWM(6, 0);
        LEDControl.setPWM(7, 0);
        LEDControl.setPWM(10, 0);
        LEDControl.setPWM(11, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 8000) {
      if (LEDControl.getPWM(2) == 0) {
        LEDControl.setPWM(2, ledMaxBright * 3 / 4);
        LEDControl.setPWM(3, ledMaxBright * 3 / 4);
        LEDControl.setPWM(6, ledMaxBright * 3 / 4);
        LEDControl.setPWM(7, ledMaxBright * 3 / 4);
        LEDControl.setPWM(10, ledMaxBright * 3 / 4);
        LEDControl.setPWM(11, ledMaxBright * 3 / 4);
        LEDControl.write();
      }
    }
    else {
      MP3Trigger.stop();
      routineSongPlaying = false;
    }
  }

  else if (curTime < 10100) {
    if (curTime < 8500) {
      if (!routineSongPlaying) {
        MP3Trigger.setVolume(15);
        MP3Trigger.trigger(53); // Lights turning on
      }  
    }

    // Flicker lights on
    else if (curTime < 8800) {
      if (LEDControl.getPWM(14) == 0) {
        LEDControl.setPWM(14, ledMaxBright / 4);
        LEDControl.setPWM(15, ledMaxBright / 4);
        LEDControl.setPWM(18, ledMaxBright / 4);
        LEDControl.setPWM(19, ledMaxBright / 4);
        LEDControl.setPWM(22, ledMaxBright / 4);
        LEDControl.setPWM(23, ledMaxBright / 4);
        LEDControl.write();
      }
    }
    else if (curTime < 9100) {
      if (LEDControl.getPWM(14) != 0) {
        LEDControl.setPWM(14, 0);
        LEDControl.setPWM(15, 0);
        LEDControl.setPWM(18, 0);
        LEDControl.setPWM(19, 0);
        LEDControl.setPWM(22, 0);
        LEDControl.setPWM(23, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 9400) {
      if (LEDControl.getPWM(14) == 0) {
        LEDControl.setPWM(14, ledMaxBright / 2);
        LEDControl.setPWM(15, ledMaxBright / 2);
        LEDControl.setPWM(18, ledMaxBright / 2);
        LEDControl.setPWM(19, ledMaxBright / 2);
        LEDControl.setPWM(22, ledMaxBright / 2);
        LEDControl.setPWM(23, ledMaxBright / 2);
        LEDControl.write();
      }
    }
    else if (curTime < 9700) {
      if (LEDControl.getPWM(14) != 0) {
        LEDControl.setPWM(14, 0);
        LEDControl.setPWM(15, 0);
        LEDControl.setPWM(18, 0);
        LEDControl.setPWM(19, 0);
        LEDControl.setPWM(22, 0);
        LEDControl.setPWM(23, 0);
        LEDControl.write();
      }
    }
    else if (curTime < 10000) {
      if (LEDControl.getPWM(14) == 0) {
        LEDControl.setPWM(14, ledMaxBright * 3 / 4);
        LEDControl.setPWM(15, ledMaxBright * 3 / 4);
        LEDControl.setPWM(18, ledMaxBright * 3 / 4);
        LEDControl.setPWM(19, ledMaxBright * 3 / 4);
        LEDControl.setPWM(22, ledMaxBright * 3 / 4);
        LEDControl.setPWM(23, ledMaxBright * 3 / 4);
        LEDControl.write();
      }
    }

    else {
      MP3Trigger.stop();
      routineSongPlaying = false;
    }
  }

}

void clubDancing() {
  long curTime = millis() - subRoutineStart;
  
  if (curTime < 500) {
    if (!routineSongPlaying) {
      Serial.println("Starting crowd noise");
      MP3Trigger.setVolume(5);
      MP3Trigger.trigger(54); // Play crowd noise
      routineSongPlaying = true;
    }
    
    if (!doorOpen) {
      openDoor(1);
      //Serial.println("Open door");
    }
  }
  else if (curTime < 1500) {
    if (LEDControl.getPWM(2) == ledMaxBright * 3 / 4) {
      LEDControl.setPWM(2, ledMaxBright / 2);
      LEDControl.setPWM(3, ledMaxBright / 2);
      LEDControl.setPWM(6, ledMaxBright / 2);
      LEDControl.setPWM(7, ledMaxBright / 2);
      LEDControl.setPWM(10, ledMaxBright / 2);
      LEDControl.setPWM(11, ledMaxBright / 2);
      LEDControl.setPWM(14, ledMaxBright / 2);
      LEDControl.setPWM(15, ledMaxBright / 2);
      LEDControl.setPWM(18, ledMaxBright / 2);
      LEDControl.setPWM(19, ledMaxBright / 2);
      LEDControl.setPWM(22, ledMaxBright / 2);
      LEDControl.setPWM(23, ledMaxBright / 2);
      LEDControl.write();
    }
  }
  else if (curTime < 2500) {
    if (LEDControl.getPWM(2) == ledMaxBright / 2) {
      LEDControl.setPWM(2, ledMaxBright / 4);
      LEDControl.setPWM(3, ledMaxBright / 4);
      LEDControl.setPWM(6, ledMaxBright / 4);
      LEDControl.setPWM(7, ledMaxBright / 4);
      LEDControl.setPWM(10, ledMaxBright / 4);
      LEDControl.setPWM(11, ledMaxBright / 4);
      LEDControl.setPWM(14, ledMaxBright / 4);
      LEDControl.setPWM(15, ledMaxBright / 4);
      LEDControl.setPWM(18, ledMaxBright / 4);
      LEDControl.setPWM(19, ledMaxBright / 4);
      LEDControl.setPWM(22, ledMaxBright / 4);
      LEDControl.setPWM(23, ledMaxBright / 4);
      LEDControl.write();
    }
  }
  else if (curTime < 3500) {
    if (LEDControl.getPWM(2) == ledMaxBright / 4) {
      clearLights();
    }

    if (curTime > 3000) {
      if (routineSongPlaying) {
        MP3Trigger.stop();
        routineSongPlaying = false;
      }

      if (doorOpen) {
        closeDoor(1);
        //Serial.println("Open door");
      }
    }
  }

  else if (curTime < 13000) {
    if (curTime < 4000 && !routineSongPlaying) {
      Serial.println("Starting club music");
      MP3Trigger.setVolume(15);
      MP3Trigger.trigger(55); // Play club music
      routineSongPlaying = true;
    }

    if (curTime % 200 < 20) {
        if (!switchMade) {
          Serial.println("curTime: " + String(curTime) + "; curLight: " + String(curLight) + "; nextLight: " + String(nextLight));
          if (curTime < 8500) {
            if (nextLight % 4 < 3) {
              LEDControl.setPWM(nextLight, ledMaxBright / 2);
              LEDControl.setPWM(nextLight + 1, ledMaxBright / 2);
            }
            else {
              LEDControl.setPWM(nextLight, ledMaxBright / 2);
              LEDControl.setPWM(nextLight - 3, ledMaxBright / 2);
            }
            if (curLight % 4 < 3) {
              LEDControl.setPWM(curLight, 0);
              LEDControl.setPWM(curLight + 1, 0);
            }
            else {
              LEDControl.setPWM(curLight, 0);
              LEDControl.setPWM(curLight - 3, 0);
            }
            curLight = nextLight;
            nextLight = (nextLight < 20) ? (nextLight + 4) : ((nextLight + 1) % 4);
          }
          else {
            LEDControl.setPWM(nextLight, ledMaxBright / 2);
            LEDControl.setPWM(nextLight + 1, ledMaxBright / 2);
            LEDControl.setPWM(nextLight + 2, ledMaxBright / 2);
            LEDControl.setPWM(nextLight + 3, ledMaxBright / 2);
            LEDControl.setPWM(curLight, 0);
            LEDControl.setPWM(curLight + 1, 0);
            LEDControl.setPWM(curLight + 2, 0);
            LEDControl.setPWM(curLight + 3, 0);
            curLight = nextLight;
            nextLight = (nextLight < 20) ? (nextLight + 4) : ((nextLight) % 4);
          }
          LEDControl.write();
          switchMade = true;
        }
    }
    else {
      if (switchMade) {
        switchMade = false;
      }
    }

    // Dancing to club music
    if (curTime < 4500) {
      ST->drive(-50 * r2speedBoost);
      ST->turn(0);
    }
    else if (curTime < 5500) {
      ST->drive(50 * r2speedBoost);
      ST->turn(0);
    }
    else if (curTime < 6000) {
      ST->drive(0);
      ST->turn(-60 * r2turnBoost);
    }
    else if (curTime < 6800) {
      ST->drive(0);
      ST->turn(60 * r2turnBoost);
    }
    else if (curTime < 12000) {
      ST->drive(0);
      ST->turn(-60 * r2turnBoost);
    }
    else if (routineSongPlaying) {
      ST->turn(0);
      MP3Trigger.stop();
      routineSongPlaying = false;
    }
  }
}

void playClosingTime() {
  long curTime = millis() - subRoutineStart;

  // Lighting
  if (curTime < 11000) {
    if ((curTime % 4000) < 2000) {
      if (LEDControl.getPWM(0) == 0) {
        for (int i = 0; i < 12; i++) {
          LEDControl.setPWM(i, ledMaxBright);
        }
        for (int i = 12; i < 24; i++) {
          LEDControl.setPWM(i, 0);
        }
        LEDControl.write();
      }
    }
    else {
      if (LEDControl.getPWM(12) == 0) {
        for (int i = 0; i < 12; i++) {
          LEDControl.setPWM(i, 0);
        }
        for (int i = 12; i < 24; i++) {
          LEDControl.setPWM(i, ledMaxBright);
        }
        LEDControl.write();
      }
    }
  }
  else if (curTime < 14000) {
    if (LEDControl.getPWM(12) != 0) {
      clearLights();
    }
  }
  else {
    if (LEDControl.getPWM(12) == 0) {
      for (int i = 0; i < 24; i++) {
          LEDControl.setPWM(i, ledMaxBright);
      }
      LEDControl.write();
    }
  }

  // Trigger sound
  if (curTime < 800) {
    if (!routineSongPlaying) {
      MP3Trigger.setVolume(25);
      MP3Trigger.trigger(56); // Play Closing Time
      routineSongPlaying = true;
    }
  }

  // Driving
  if (curTime > 6000 && curTime < 9000) {
    ST->drive(-30 * r2speedBoost);
  }
  else if (curTime < 16000) {
    ST->drive(0);
  }
  else if (curTime < 17200) {
    ST->turn(-30 * r2turnBoost);
  }
  else if (curTime < 19000) {
    ST->turn(30 * r2turnBoost);
  }
  else {
    ST->drive(0);
    ST->turn(0);
  }
  

  // Servo and end of audio
  if (curTime > 3500) {
    if (curTime < 11400) {
      if (!doorOpen) {
        Serial.println("Open door");
        openDoor(1);
      }
    }
    else if (curTime < 19000) {
      // Close door
      if (doorOpen) {
        Serial.println("Close door");
        closeDoor(1);
      }
    }
    else if (curTime < 21000) {
      // Open door
      if (!doorOpen) {
        Serial.println("Open door");
        openDoor(1);
        MP3Trigger.stop();
      }
    }
    else if (curTime < 22000) {
      // Close door
      if (doorOpen) {
        Serial.println("Close door");
        closeDoor(2);
        MP3Trigger.setVolume(5);
        MP3Trigger.trigger(57); // Slam door
        clearLights();
      }
    }
  }
}


void openDoor(int factor) {
  int curLoc = myServo.read();

  if (curLoc < 179) {
    myServo.write(curLoc + factor);
  }
  else if (curLoc == 179) {
    myServo.write(180);
  }

  if (myServo.read() == 180) {
    doorOpen = true;  
  }

  //Serial.println("Angle: " + String(myServo.read()));
}


void closeDoor(int factor) {
  int curLoc = myServo.read();

  if (curLoc > 116) {
    myServo.write(curLoc - factor);
  }
  else if (curLoc == 116) {
    myServo.write(115);
  }

  if (myServo.read() == 115) {
    doorOpen = false;  
  }

  //Serial.println("Angle: " + String(myServo.read()));
}


void routine3() {
  long curTime = millis() - routineStart - 100; // -100ms for song start delay
  float measure = routine3mspb * 16.0;
  
  if (!routineSongPlaying) {
    MP3Trigger.trigger(60); // Play Cotton Eye Joe
    clearLights();
    routineSongPlaying = true;
  }

  if (curTime < measure * 2) {
    if (curSubRoutine != 1) {
      Serial.println("Playing routine 3 subroutine 1");
      subRoutineStart = millis();
      curSubRoutine = 1;
    }
    introRoutine3();
  }
  else if (curTime < measure * 3) {
    if (curSubRoutine != 2) {
      Serial.println("Playing routine 3 subroutine 2");
      subRoutineStart = millis();
      curSubRoutine = 2;
    }
    cottonEyeJoe(1.0);
  }
  else if (curTime < measure * 4) {
    if (curSubRoutine != 3) {
      Serial.println("Playing routine 3 subroutine 3");
      subRoutineStart = millis();
      curSubRoutine = 3;
    }
    cottonEyeJoe(1.5);
  }
  else if (curTime < measure * 5) {
    if (curSubRoutine != 4) {
      Serial.println("Playing routine 3 subroutine 4");
      subRoutineStart = millis();
      curSubRoutine = 4;
    }
    cottonEyeJoeFAST();
  }
  else if (curTime > measure * 6) {
    Serial.println(millis());
    Serial.println(routineStart);
    Serial.println("Setting routineNumber back to 0");
    routineNumber = 0;
    curSubRoutine = 0;
    routineSongPlaying = false;
    clearLights();
    MP3Trigger.setVolume(BASE_VOLUME);
  }
}

// Runs for 2 measures (32 BPM intervals)
void introRoutine3() {
  long curTime = millis() - subRoutineStart;

  // Measure 1 (Lights only)
  if (curTime < routine3mspb * 2) {
    clearLights();
    setLightColor("red", ledMaxBright/10);
  } else if (curTime < routine3mspb * 4) {
    setLightColor("red", 0);
    setLightColor("yellow", ledMaxBright/10);
  } else if (curTime < routine3mspb * 6) {
    setLightColor("yellow", 0);
    setLightColor("green", ledMaxBright/10);
  } else if (curTime < routine3mspb * 8) {
    setLightColor("green", 0);
    setLightColor("blue", ledMaxBright/10);
  } else if (curTime < routine3mspb * 9) {
    setLightColor("blue", 0);
    setLightColor("red", ledMaxBright/4);
  } else if (curTime < routine3mspb * 10) {
    setLightColor("red", 0);
    setLightColor("yellow", ledMaxBright/4);
  } else if (curTime < routine3mspb * 11) {
    setLightColor("yellow", 0);
    setLightColor("green", ledMaxBright/4);
  } else if (curTime < routine3mspb * 12) {
    setLightColor("green", 0);
    setLightColor("blue", ledMaxBright/4);
  } else if (curTime < routine3mspb * 13) {
    setLightColor("blue", 0);
    setLightColor("red", ledMaxBright);
  } else if (curTime < routine3mspb * 14) {
    setLightColor("red", 0);
    setLightColor("yellow", ledMaxBright);
  } else if (curTime < routine3mspb * 15) {
    setLightColor("yellow", 0);
    setLightColor("green", ledMaxBright);
  } else if (curTime < routine3mspb * 16) {
    setLightColor("green", 0);
    setLightColor("blue", ledMaxBright);
  } else if (curTime < routine3mspb * 17) {
    setLightColor("blue", 0);
    setLightGroup(0, ledMaxBright);
  } else if (curTime < routine3mspb * 18) {
    setLightGroup(0, 0);
    setLightGroup(1, ledMaxBright);
  } else if (curTime < routine3mspb * 19) {
    setLightGroup(1, 0);
    setLightGroup(2, ledMaxBright);
  } else if (curTime < routine3mspb * 20) {
    setLightGroup(2, 0);
    setLightGroup(3, ledMaxBright);
  } else if (curTime < routine3mspb * 21) {
    setLightGroup(3, 0);
    setLightGroup(4, ledMaxBright);
  } else if (curTime < routine3mspb * 22) {
    setLightGroup(4, 0);
    setLightGroup(5, ledMaxBright);
  } else if (curTime < routine3mspb * 23) {
    setLightGroup(5, 0);
    setLightGroups(0, 2, ledMaxBright);
  } else if (curTime < routine3mspb * 24) {
    setLightGroups(0, 2, 0);
    setLightGroups(3, 5, ledMaxBright);
  } else if (curTime < routine3mspb * 25) {
    setLightGroups(0, 2, ledMaxBright);
  } else if (curTime < routine3mspb * 26) {
    setLightGroups(3, 5, 0);
  } else if (curTime < routine3mspb * 27) {
    setLightGroups(0, 2, 0);
    setLightGroup(5, ledMaxBright);
  } else if (curTime < routine3mspb * 28) {
    setLightGroup(5, 0);
    setLightGroup(4, ledMaxBright);
  } else if (curTime < routine3mspb * 29) {
    setLightGroup(4, 0);
    setLightGroup(3, ledMaxBright);
  } else if (curTime < routine3mspb * 30) {
    setLightGroup(3, 0);
    setLightGroup(2, ledMaxBright);
  } else if (curTime < routine3mspb * 31) {
    setLightGroup(2, 0);
    setLightGroup(1, ledMaxBright);
  } else if (curTime < routine3mspb * 32) {
    setLightGroup(1, 0);
    setLightGroup(0, ledMaxBright);
  }
}

void cottonEyeJoe(float factor) {
  long curTime = millis() - subRoutineStart;

  if (curTime < routine3mspb * 1) {
    clearLights();
    setLightRotation(0, ledMaxBright);
  } else if (curTime < routine3mspb * 2) {
    setLightRotation(0, 0);
    setLightRotation(1, ledMaxBright);
  } else if (curTime < routine3mspb * 3) {
    setLightRotation(1, 0);
    setLightRotation(2, ledMaxBright);
  } else if (curTime < routine3mspb * 4) {
    setLightRotation(2, 0);
    setLightRotation(3, ledMaxBright);
  } else if (curTime < routine3mspb * 5) {
    setLightRotation(3, 0);
    setLightRotation(0, ledMaxBright);
  }  else if (curTime < routine3mspb * 6) {
    setLightRotation(0, 0);
    setLightRotation(1, ledMaxBright);
  } else if (curTime < routine3mspb * 7) {
    setLightRotation(1, 0);
    setLightRotation(2, ledMaxBright);
  } else if (curTime < routine3mspb * 8) {
    setLightRotation(2, 0);
    setLightRotation(3, ledMaxBright);
  } else if (curTime < routine3mspb * 9) {
    setLightRotation(3, 0);
    setLightRotation(0, ledMaxBright);
  } else if (curTime < routine3mspb * 10) {
    setLightRotation(0, 0);
    setLightRotation(1, ledMaxBright);
  } else if (curTime < routine3mspb * 11) {
    setLightRotation(1, 0);
    setLightRotation(2, ledMaxBright);
  } else if (curTime < routine3mspb * 12) {
    setLightRotation(2, 0);
    setLightRotation(3, ledMaxBright);
  } else if (curTime < routine3mspb * 13) {
    setLightRotation(3, 0);
    setLightRotation(0, ledMaxBright);
  }  else if (curTime < routine3mspb * 14) {
    setLightRotation(0, 0);
    setLightRotation(1, ledMaxBright);
  } else if (curTime < routine3mspb * 15) {
    setLightRotation(1, 0);
    setLightRotation(2, ledMaxBright);
  } else if (curTime < routine3mspb * 16) {
    setLightRotation(2, 0);
    setLightRotation(3, ledMaxBright);
  } 
}

void cottonEyeJoeFAST() {
  long curTime = millis() - subRoutineStart;
  float routine3mspbfast = routine3mspb/2; // 32 per measure

  if (curTime < routine3mspbfast * 1) {
    clearLights();
    setLightColor("red", ledMaxBright);
  } else if (curTime < routine3mspbfast * 2) {
    setLightColor("red", 0);
    setLightColor("yellow", ledMaxBright);
  } else if (curTime < routine3mspbfast * 3) {
    setLightColor("yellow", 0);
    setLightColor("green", ledMaxBright);
  } else if (curTime < routine3mspbfast * 4) {
    setLightColor("green", 0);
    setLightColor("blue", ledMaxBright);
  } else if (curTime < routine3mspbfast * 5) {
    setLightColor("blue", 0);
    setLightColor("red", ledMaxBright);
  } else if (curTime < routine3mspbfast * 6) {
    setLightColor("red", 0);
    setLightColor("yellow", ledMaxBright);
  } else if (curTime < routine3mspbfast * 7) {
    setLightColor("yellow", 0);
    setLightColor("green", ledMaxBright);
  } else if (curTime < routine3mspbfast * 8) {
    setLightColor("green", 0);
    setLightColor("blue", ledMaxBright);
  } else if (curTime < routine3mspbfast * 9) {
    setLightColor("blue", 0);
    setLightColor("red", ledMaxBright);
  } else if (curTime < routine3mspbfast * 10) {
    setLightColor("red", 0);
    setLightColor("yellow", ledMaxBright);
  } else if (curTime < routine3mspbfast * 11) {
    setLightColor("yellow", 0);
    setLightColor("green", ledMaxBright);
  } else if (curTime < routine3mspbfast * 12) {
    setLightColor("green", 0);
    setLightColor("blue", ledMaxBright);
  } else if (curTime < routine3mspbfast * 13) {
    setLightColor("blue", 0);
    setLightColor("red", ledMaxBright);
  } else if (curTime < routine3mspbfast * 14) {
    setLightColor("red", 0);
    setLightColor("yellow", ledMaxBright);
  } else if (curTime < routine3mspbfast * 15) {
    setLightColor("yellow", 0);
    setLightColor("green", ledMaxBright);
  } else if (curTime < routine3mspbfast * 16) {
    setLightColor("green", 0);
    setLightColor("blue", ledMaxBright);
  } else if (curTime < routine3mspbfast * 17) {
    setLightColor("blue", 0);
    setLightGroup(0, ledMaxBright);
  } else if (curTime < routine3mspbfast * 18) {
    setLightGroup(0, 0);
    setLightGroup(1, ledMaxBright);
  } else if (curTime < routine3mspbfast * 19) {
    setLightGroup(1, 0);
    setLightGroup(2, ledMaxBright);
  } else if (curTime < routine3mspbfast * 20) {
    setLightGroup(2, 0);
    setLightGroup(3, ledMaxBright);
  } else if (curTime < routine3mspbfast * 21) {
    setLightGroup(3, 0);
    setLightGroup(4, ledMaxBright);
  } else if (curTime < routine3mspbfast * 22) {
    setLightGroup(4, 0);
    setLightGroup(5, ledMaxBright);
  } else if (curTime < routine3mspbfast * 23) {
    setLightGroup(5, 0);
    setLightGroups(0, 2, ledMaxBright);
  } else if (curTime < routine3mspbfast * 24) {
    setLightGroups(0, 2, 0);
    setLightGroups(3, 5, ledMaxBright);
  } else if (curTime < routine3mspbfast * 25) {
    setLightGroups(3, 5, 0);
  } else if (curTime < routine3mspbfast * 26) {
    setLightGroups(0, 2, 0);
    setLightGroup(5, ledMaxBright);
  } else if (curTime < routine3mspbfast * 27) {
    setLightGroup(5, 0);
    setLightGroup(4, ledMaxBright);
  } else if (curTime < routine3mspbfast * 28) {
    setLightGroup(4, 0);
    setLightGroup(3, ledMaxBright);
  } else if (curTime < routine3mspbfast * 29) {
    setLightGroup(3, 0);
    setLightGroup(2, ledMaxBright);
  } else if (curTime < routine3mspbfast * 30) {
    setLightGroup(2, 0);
    setLightGroup(1, ledMaxBright);
  } else if (curTime < routine3mspbfast * 31) {
    setLightGroup(1, 0);
    setLightGroups(0, 5, ledMaxBright);
  }
}

void setLightColor(String color, int brightness) {
  bool changeMade = false;
  
  for (int i = parseColor(color); i < 24; i += 4) LEDControl.setPWM(i, brightness);
  LEDControl.write();
}

void setLightGroup(int group, int brightness) {
  for (int i = group*4; i < group*4 + 4; i++) LEDControl.setPWM(i, brightness);
  LEDControl.write();
}

void setLightGroups(int first, int last, int brightness) {
  for (int i = first; i < last + 1; i++) setLightGroup(i, brightness);
}

void setLightRotation(int x, int brightness) {
  for (int i = 0; i < 3; i++) {
    LEDControl.setPWM(((x + i*5) % 4) + i*4, brightness);
    LEDControl.setPWM(((x + i*5) % 4) + (20 - i*4), brightness);
  }

  LEDControl.write();
}

int parseColor(String color) {
  if (color == "red") 
    return 0;
  else if (color == "yellow") 
    return 1;
  else if (color == "green") 
    return 2;
  else if (color == "blue") 
    return 3;
  else 
    return 20; // Turn on one red light as an error
}

void clearLights() {
  for (int i = 0; i < 24; i++) {
    LEDControl.setPWM(i, 0);
  }
  LEDControl.write();
}


// =======================================================================================
//      YOUR CUSTOM DROID FUNCTIONS SHOULD END HERE
// =======================================================================================

// =======================================================================================
//      CORE DROID CONTROL FUNCTIONS START HERE - EDIT WITH CAUTION
// =======================================================================================
// Read the PS3 Controller and set request state variables
void readPS3Request() {
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
void resetRequestVariables() {
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
