//FINAL PROJECT!!!
//RBE 1001 Team #10
//Megan DeSanty, Jacob Chlebowski, Aiden Veccia

#include <RBE1001Lib.h>
#include <IRdecoder.h>
#include <RemoteConstants.h>
#include <ESP32AnalogRead.h>

//sets up the left and right sensors used in line following
ESP32AnalogRead leftSensor;
ESP32AnalogRead rightSensor;

//sets up the IR sensor
const uint8_t IR_DETECTOR_PIN = 15;
IRDecoder decoder(IR_DETECTOR_PIN);

//Declares the left and right motor
LeftMotor left;
RightMotor right;

//Sets up the ultrasonic sensor
Rangefinder ultrasonic;

//Calculations for the drive and turn function
const float diameter = 7;
const float degreesPerCM = 360.0 / (3.14 * diameter);
const float track = 14.0;
const float wheelRotPerRootRot = track / diameter;

//Declares the servo pin
int servoPin = 33;
//Declares the servo
Servo armServo;

//Boolean that determines whether the robot has picked up the bag specifically in the searching function
bool itemGet = false; //This value is only false for the robot that goes into the free range and is true for the other robots

//Delay variables used at intersections
//sDelay stands for straight delay and it is used when we need our robot to go straight over an intersection
int sDelay = 200;
//tDelay stands for turn delay and it is used when the robot has a bag and is about to do a 90 degree turn
int tDelay = 350; //This value is different for each robot due to their different weights
//etDelay stands for turn delay and it is used when the robot does not have a bag and is about to do a 90 degree turn
int etDelay = 300; //This value is different for each robot due to their different weights

//This variable stops the robot from

//This variable represents how many cups have been deposited and tells the robot which zone to go to next
int cupCount = 0; //This value will be different for each robot because they all go to different zones
//This variable becomes true every time a robot picks up a cup, and becomes false when the robot drops it
bool hasCup = false;
//Each state represents a section of the track, Z1 Z2 and Z3 represent the sections of track in front of each of the dropoff zones
typedef enum State
{
  RA,
  RB,
  RC,
  RD,
  RE,
  RF,
  Z1,
  Z2,
  Z3,
  //Search and spin do not represent sections of track, but are instead used to find the free range bag
  search,
  spin
} State;
//This is the state that all robots begin in
State state = RA;

//Variables used in searching function
float degreeCount = 0;
float totalDegree = 0;

//Setup function
void setup()
{
  Serial.begin(115200);
  decoder.init();
  armServo.attach(servoPin);
  leftSensor.attach(36);
  rightSensor.attach(39);
  ultrasonic.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  //This ensures that the robot's arm will always start at it's lowest position
  armServo.write(0);
}

//Function used to turn the robots
void turn(float degrees)
{
  left.startMoveFor(-degrees * wheelRotPerRootRot, 180);
  right.moveFor(degrees * wheelRotPerRootRot, 180);
}

//This function was used when we needed a robot to continue line following for a specific ammount of time
void lineFollowDelay(float time)
{
  for (time; time >= 1; time -= 0.5)
  {
    float Vleft = leftSensor.readVoltage();
    float Vright = rightSensor.readVoltage();
    float error = 0.03 * (Vleft - Vright);

    left.setEffort(0.2 - error);
    right.setEffort(0.2 + error);
  }
}

void loop()
{
  //This activates the IR sensor so we can usie it later in the loop
  int16_t keyPress = decoder.getKeyCode();

  float Vleft = leftSensor.readVoltage();
  float Vright = rightSensor.readVoltage();
  float error = 0.03 * (Vleft - Vright);

  //All commented print statements were used during testing
  //Serial.println("Vleft:");
  //Serial.println(Vleft);
  //Serial.println("Vright:");
  //Serial.println(Vright);

  //Line following it always happening unless a block is used such as a while loop
  left.setEffort(0.2 - error);
  right.setEffort(0.213 + error);

  switch (state)
  {
  case RA:
    //Serial.println("RA");
    if (left.getCurrentDegrees() >= 900 && itemGet == false)
    {
      state = spin;
    }
    //This kind of if statement is used to detect when the robot crosses a perpendicular line, it is used many times after this
    else if (Vleft >= 1.7 and Vright >= 1.7)
    {
      //If the robot has the cup
      if (hasCup == true)
      {
        //Go to Road B
        if (ultrasonic.getDistanceCM() <= 30)
        {
          delay(tDelay);
          turn(-90);
          state = RB;
        }
        //Go to road E
        else
        {
          delay(sDelay);
          state = RE;
        }
      }
      //Go to road F
      else
      {
        delay(sDelay);
        state = RF;
      }
    }
    break;

  case RB:
    //Serial.println("RB");
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
    //Go to Road C
      if (hasCup == true)
      {
        delay(tDelay);
        turn(90);
        state = RC;
      }
      //Go to road A
      else
      {
        delay(etDelay);
        turn(90);
        state = RA;
      }
    }
    break;
  case RC:
    //Serial.println("RC");
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
      if (hasCup == true)
      {
          //Go down road D
        if (cupCount < 2)
        {
          delay(tDelay);
          turn(90);
          state = RD;
        }
        //Enter Zone 3
        else
        {
          delay(tDelay);
          turn(-90);
          state = Z3;
        }
      }
      //Go to road B
      else
      {
        delay(etDelay);
        turn(-90);
        state = RB;
      }
    }
    break;

  case RD:
    //Serial.println("RD");
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
      if (hasCup == true)
      {
        if (cupCount < 2)
        {
            //Go to zone 1
          if (cupCount == 0)
          {
            delay(sDelay);
            state = Z1;
          }
          //Go to zone 2
          else
          {
            delay(tDelay);
            turn(-90);
            state = Z2;
          }
        }
        //Go to zone 3
        else
        {
          delay(sDelay);
          state = Z3;
        }
      }
      //Go to road C
      else
      {
        delay(etDelay);
        turn(-90);
        state = RC;
      }
    }
    break;

  case RE:
    //Serial.println("RE");
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
      if (hasCup == true)
      {
          //Go to zone 1
        if (cupCount == 0)
        {
          delay(tDelay);
          turn(90);
          state = Z1;
        }
        //Go to zone 2
        if (cupCount == 1)
        {
          delay(sDelay);
          state = Z2;
        }
        //Go to road D
        if (cupCount == 2)
        {
          delay(tDelay);
          turn(-90);
          state = RD;
        }
      }
      //Go to road A
      else
      {
        delay(sDelay);
        state = RA;
      }
    }
    break;

  case RF:
    //Serial.println("RF");
    while (hasCup == false)
    {
        //Picks up the bag at the end of road F
      while (ultrasonic.getDistanceCM() >= 12)
      {
        left.setEffort(0.2);
        right.setEffort(0.213);
      }
      turn(180);
      left.setEffort(-0.1);
      right.setEffort(-0.1);
      delay(900);
      armServo.write(180);
      hasCup = true;
    }
    //go to road A
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
      delay(sDelay);
      state = RA;
    }
    break;

  case Z1:
    //Serial.println("Z1");
      //Drops the bag in zone 1
    while (hasCup == true)
    {
      lineFollowDelay(2700);
      turn(180);
      armServo.write(0);
      delay(500);
      cupCount = 1;
      hasCup = false;
    }
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
      delay(etDelay);
      turn(-90);
      //go to road D
      if (ultrasonic.getDistanceCM() <= 30)
      {
        turn(90);
        state = RD;
      }
      //Go to road E
      else
      {
        state = RE;
      }
    }
    break;

  case Z2:
    //Serial.println("Z2");
      //Drop the cup in zone 2
    while (hasCup == true)
    {
      lineFollowDelay(1450);
      turn(180);
      armServo.write(70);
      delay(500);
      armServo.write(0);
      cupCount = 2;
      hasCup = false;
    }
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
        //Go to road D
      if (ultrasonic.getDistanceCM() <= 30)
      {
        delay(etDelay);
        turn(90);
        state = RD;
      }
      //Go to road E
      else
      {
        delay(sDelay);
        state = RE;
      }
    }
    break;

  case Z3:
    //Serial.println("Z3");
    while (hasCup == true)
    {
        //Drops the cup in zone 3
      lineFollowDelay(3000);
      turn(180);
      armServo.write(50);
      delay(500);
      armServo.write(0);
      cupCount = 0;
      hasCup = false;
    }
    if (Vleft >= 1.7 and Vright >= 1.7)
    {
        //Go to road C
      delay(etDelay);
      turn(90);
      state = RC;
    }
    break;

  case search:
    //Serial.println("search");
      //Keeps driving
    while (ultrasonic.getDistanceCM() >= 12 and itemGet == false)
    {
      left.setEffort(0.2);
      right.setEffort(0.213);
    }
    //Gets the free range bag
    if (itemGet == false)
    {
      turn(180);
      left.setEffort(-0.2);
      right.setEffort(-0.2);
      delay(450);
      armServo.write(180);
      turn(65 + totalDegree);
      itemGet = true;
    }
    //Getting back to the line
    if (Vleft >= 1.7 && Vright >= 1.7)
    {
      delay(tDelay);
      turn(-95);
      hasCup = true;
      state = RA;
    }
    //enter road A
    else if (Vleft >= 1.7 and Vright <= 1.7)
    {
      delay(tDelay);
      turn(-75);
      hasCup = true;
      state = RA;
    }
    //enter road A after turning
    else if (Vleft <= 1.7 and Vright >= 1.7)
    {
      delay(tDelay);
      turn(-115);
      hasCup = true;
      state = RA;
    }
    else
    {
      left.setEffort(0.2);
      right.setEffort(0.213);
    }
    break;

  case spin:
    turn(70);
    //slowly search for the bag
    while (ultrasonic.getDistanceCM() >= 50)
    {
      turn(-1);
      totalDegree -= 1.5;
    }
    //Found the bag
    while (ultrasonic.getDistanceCM() <= 50)
    {
      turn(-1);
      degreeCount += 0.55;
    }
    //enter search state
    turn(degreeCount);
    state = search;
    break;
  }
}
