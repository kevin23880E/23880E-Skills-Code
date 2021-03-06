/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FrontLeftDrive       motor         11              
// FrontRightDrive      motor         3               
// BackLeftDrive        motor         18              
// BackRightDrive       motor         5               
// LTrack               rotation      12              
// RTrack               rotation      17              
// STrack               rotation      20              
// LeftIntake           motor         1               
// RightIntake          motor         8               
// TopRoller            motor         9               
// BottomRoller         motor         10              
// RangeFinderC         sonar         C, D            
// RangeFinderE         sonar         E, F            
// Inertial6            inertial      6               
// RangeFinderA         sonar         A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "chassis-control.h"
#include "draw-field.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

task odometryTask;
task drawFieldTask;
task chassisControlTask;
task intakeTask;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Inertial6.calibrate();

  while(Inertial6.isCalibrating()) {
    task::sleep(100);
  }

  Inertial6.setHeading(180, rotationUnits::deg);

  Brain.Screen.setCursor(3, 4);
  Brain.Screen.print("CALIBRATED!");

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

bool runTheIntakes = false;
bool runTheBottomRoller = false;
bool runTheTopRoller = false;

bool ratchetIsOpen = false;

bool ratchetCloseAutomatically = true;

/*When to stop running the intakes
  0 - When I tell it to stop
  1 - When ball reaches bottom sensor
  2 - When ball reaches top sensor
  3 - When balls in both sensors
*/
int intakeStopCondition = 0;

double intakeSpeed = 1.0;
double bottomRollerSpeed = 1.0;
double topRollerSpeed = 1.0;

int totalTimeSpent = 0;

int maxAllowedTime = 0;

double leftIntakeError = 0;
double rightIntakeError = 0;

double intakekP = 0.7;

//task to control the intakes and indexers during auton
int intakeControl() {
  while(1) {
    
    if(ratchetIsOpen) {
      leftIntakeError = -45 - LeftIntake.position(rotationUnits::deg);
      rightIntakeError = -45 - RightIntake.position(rotationUnits::deg);

      LeftIntake.spin(directionType::fwd, leftIntakeError * intakekP, voltageUnits::volt);
      RightIntake.spin(directionType::fwd, rightIntakeError * intakekP, voltageUnits::volt);

      // if(RangeFinderA.distance(distanceUnits::in) > 1 && RangeFinderA.distance(distanceUnits::in) < 11 && ratchetCloseAutomatically) {
      //   ratchetIsOpen = false;
      // }
    }

    else {

      if(runTheIntakes) {
        LeftIntake.spin(directionType::fwd, intakeSpeed * 100, velocityUnits::pct);
        RightIntake.spin(directionType::fwd, intakeSpeed * 100, velocityUnits::pct);
      }
      else {
        LeftIntake.stop(brakeType::brake);
        RightIntake.stop(brakeType::brake);
      }

      //condition for reaching first checkpoint
      if(intakeStopCondition == 1 && RangeFinderC.distance(distanceUnits::in) < 4 && RangeFinderC.distance(distanceUnits::in) > 0.01) {
        runTheIntakes = false;
        runTheBottomRoller = false;
        
      }
      else if(intakeStopCondition == 2 && RangeFinderE.distance(distanceUnits::in) < 6 && RangeFinderE.distance(distanceUnits::in) > 0.01) {
        runTheIntakes = false;
        runTheBottomRoller = false;
        runTheTopRoller = false;
      }
      else if(intakeStopCondition == 3 && RangeFinderC.distance(distanceUnits::in) < 6 && RangeFinderC.distance(distanceUnits::in) > 0.01 && RangeFinderE.distance(distanceUnits::in) < 3 && RangeFinderE.distance(distanceUnits::in) > 0.01) {
        runTheIntakes = false;
        runTheBottomRoller = false;
        runTheTopRoller = false;
      }


      //timeout (only for when there is a stop condition)
      if(totalTimeSpent > maxAllowedTime && intakeStopCondition != 0) {
        runTheIntakes = false;
        runTheBottomRoller = false;
      }

      totalTimeSpent += 10;

    }

    if(runTheBottomRoller) {
      BottomRoller.spin(directionType::fwd, bottomRollerSpeed * 100, velocityUnits::pct);
    }
    else {
      BottomRoller.stop(brakeType::brake);
    }

    if(runTheTopRoller) {
      TopRoller.spin(directionType::fwd, topRollerSpeed * 100, velocityUnits::pct);
    }
    else {
      TopRoller.stop(brakeType::brake);
    }


    task::sleep(10);
  }
  return 1;
}

void runIntakes (double speed, int stopCondition = 0, int timeOutTime = 1000) {
  totalTimeSpent = 0;
  maxAllowedTime = timeOutTime;
  runTheIntakes = true;
  runTheBottomRoller = true;
  intakeSpeed = speed;
  bottomRollerSpeed = speed;
  intakeStopCondition = stopCondition;
}

void runJustIntakes(double speed, int stopCondition = 0, int timeOutTime = 1000) {
  totalTimeSpent = 0;
  maxAllowedTime = timeOutTime;
  runTheIntakes = true;
  intakeSpeed = speed;
  intakeStopCondition = stopCondition;
}

void runBottomRoller(double speed) {
  runTheBottomRoller = true;

  bottomRollerSpeed = speed;
}

void runTopRoller(double speed) {
  runTheTopRoller = true;
  topRollerSpeed = speed;
}

void runAllIntakes(double speed, int stopCondition = 0, int timeOutTime = 1000) {
  totalTimeSpent = 0;
  runTheIntakes = true;
  runTheBottomRoller = true;
  if(stopCondition != 1) {
    runTheTopRoller = true;
    topRollerSpeed = speed;
  }
  
  intakeSpeed = speed;
  bottomRollerSpeed = speed;
  
  intakeStopCondition = stopCondition;
  maxAllowedTime = timeOutTime;

}
void stopIntakes() {
  runTheIntakes = false;
  runTheBottomRoller = false;
}

void stopBottomRoller() {
  runTheBottomRoller = false;
}

void stopTopRoller() {
  runTheTopRoller = false;
}

void stopAllIntakes() {
  runTheIntakes = false;
  runTheBottomRoller = false;
  runTheTopRoller = false;
}


void scoreBall() {
  topRollerSpeed = 1.0;
  bottomRollerSpeed = 1.0;
  runTheBottomRoller = true;
  runTheTopRoller = true;

  wait(400, timeUnits::msec);
  runTheTopRoller = false;
  runTheBottomRoller = false;
}

void openRatchets(bool autoClose = true) {
  ratchetIsOpen = true;

  ratchetCloseAutomatically = autoClose;

  LeftIntake.resetPosition();
  RightIntake.resetPosition();
  
}

void closeRatchets() {
  ratchetIsOpen = false;
}

/* AUTON PROGRAMS */

//SKILLS
void autonSkills() {
  /* AUTON SKILLS */

//testing middle goal stuff
  
  //driveTo(42.8, 9, 0, 5000, 1.0);

  //task::sleep(500000);

  //drive


//BOTTOM LEFT

  //deploy hood (score BL goal)
  runTheBottomRoller = true;
  task::sleep(400);
  runTheBottomRoller = false;

  driveTo(55, 18, 2.78, 600, 1.0);

  waitUntil(runChassisControl == false);

  driveTo(10, 35, currentAbsoluteOrientation, 1400, 1.0);

  openRatchets();
  task::sleep(650);
  closeRatchets();

  runIntakes(1.0, 2, 2000);

  task::sleep(400);

  openRatchets();
  task::sleep(350);
  closeRatchets();

  runIntakes(1.0, 3, 2000);

//line up to BL goal
  driveTo(20, 20, 5 * M_PI_4, 700, 1.0);
  waitUntil(runChassisControl == false);
//drive up to BL goal and score
  driveTo(8, 8, 5 * M_PI_4, 700, 1.0);

  //runJustIntakes(1.0, 0, 1000);
  waitUntil(runChassisControl == false);

  // runAllIntakes(1.0);

  // task::sleep(900);
 
  // stopAllIntakes();

  runAllIntakes(1.0);

  task::sleep(700);

  stopAllIntakes();

  openRatchets();

  task::sleep(300);

  closeRatchets();
  runIntakes(1.0, 3, 800);
 
  waitUntil(runTheIntakes == false);

//MIDDLE LEFT

//back up from BL goal
  driveTo(24, 40, 5 * M_PI_4, 800, 1.0);
  openRatchets();
  task::sleep(200);

//release balls

  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  waitUntil(runChassisControl == false);

  task::sleep(300);

  stopBottomRoller();
  stopTopRoller();

  closeRatchets();

//turn toward leftmost auton line ball
  turnTo(M_PI_2, 700);

  waitUntil(runChassisControl == false);

  driveTo(24, 72, M_PI_2, 1700, 1.0);

  task::sleep(600);

  openRatchets();
  task::sleep(400);
  closeRatchets();

  runIntakes(1.0, 2, 1500);

  // turnTo(3 * M_PI_4, 200);
  // task::sleep(200);
  // turnTo(M_PI, 500);
  driveTo(24, 70, M_PI, 700, 1.0);
  waitUntil(runChassisControl == false);


//score mid left goal
  driveTo(7, 70, M_PI, 600, 1.0);
  waitUntil(runChassisControl == false);

  runAllIntakes(1.0);
  task::sleep(700);

  stopAllIntakes();

  driveTo(27, 70, M_PI, 800, 1.0);

  runJustIntakes(1.0, 2, 1000);
  waitUntil(runChassisControl == false);

  turnTo(5 * M_PI_4, 500);

  //release blue ball
  openRatchets();
  runBottomRoller(-1.0);
  runTopRoller(-1.0);
  task::sleep(500);
  stopBottomRoller();
  stopTopRoller();

//turn toward middle goal
  turnTo(0, 800);

  waitUntil(runChassisControl == false);

  driveTo(73, 70, 0, 1700, 0.7);

  task::sleep(600);
  closeRatchets();

  runIntakes(1.0, 2, 2000);

  task::sleep(200);

//open ratchets to wrap middle goal
  openRatchets();

  waitUntil(runChassisControl == false);
  task::sleep(200);
//close ratchets to wrap middle goal
  closeRatchets();

//score 1 red and descore 3 blue
  //descore 1 ball
  runJustIntakes(1.0);
  task::sleep(800);
  runTheIntakes = false;
  //score the red ball
  scoreBall();

  runIntakes(1.0, 3, 1200);

  waitUntil(runTheIntakes == false);

  // runAllIntakes(0.5);
  // task::sleep(500);
  // stopAllIntakes();

  runJustIntakes(1.0, 1, 1000);
  waitUntil(runTheIntakes == false);

  openRatchets();

//back away from middle goal
  driveTo(30, 70, 0, 900, 1.0);
  waitUntil(runChassisControl == false);

  turnTo(5 * M_PI_4, 800);
  waitUntil(runChassisControl == false);

  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  task::sleep(800);

  stopBottomRoller();
  stopTopRoller();

  closeRatchets();
//TOP LEFT
//turn toward left side opposite side home row ball
  turnTo(M_PI_2, 900);

  waitUntil(runChassisControl == false);
//collect oppostie side home row ball
  driveTo(32, 115, M_PI_2, 1300, 1.0);

  task::sleep(500);

  openRatchets();

  task::sleep(800);

  closeRatchets();

  runIntakes(1.0, 2, 2500);

  waitUntil(runChassisControl == false);

//drive up to TL goal

  driveTo(20, 115, 3 * M_PI_4, 600, 1.0);

  waitUntil(runChassisControl == false);
  runJustIntakes(1.0);
  driveTo(5, 131, 3 * M_PI_4, 700, 1.0);
  waitUntil(runChassisControl == false);

  runAllIntakes(1.0);

  task::sleep(700);

  stopAllIntakes();

  openRatchets();

  task::sleep(300);

  closeRatchets();
  runIntakes(1.0, 3, 800);
 
  waitUntil(runTheIntakes == false);

  // runIntakes(1.0, 2, 1500);

  // scoreBall();

  // waitUntil(runTheIntakes == false);

  // openRatchets();
  // task::sleep(400);
  // closeRatchets();

  // runIntakes(1.0, 3, 1500);

  // waitUntil(runTheIntakes == false);


//Top Mid Goal

//get rid of 2 blue
  openRatchets();
  driveTo(35, 105, M_PI_2, 1000, 1.0);

  task::sleep(300);
  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  task::sleep(700);

  stopBottomRoller();
  stopTopRoller();

  waitUntil(runChassisControl == false);

  closeRatchets();
//line up with ball against wall
  turnTo(M_PI, 700);
  waitUntil(runChassisControl == false);

  driveTo(6, 103, M_PI, 1000, 1.0);
  openRatchets();

  waitUntil(runChassisControl == false);

  closeRatchets();
  runIntakes(1.0, 2, 2000);

//line up to ball above middle goal

  driveTo(40, 90, M_PI, 1500, 1.0);

  task::sleep(500);

  driveTo(40, 90, 0, 1000, 1.0);

  waitUntil(runChassisControl == false);

  //pick up ball above middle goal
  driveTo(70, 90, 0, 1000, 1.0);
  openRatchets();

  task::sleep(1000);

  closeRatchets();

  runIntakes(1.0, 3, 2000);

  waitUntil(runChassisControl == false);

  driveTo(67, 127, M_PI_2, 1500, 1.0);

  waitUntil(runChassisControl == false);

  runAllIntakes(1.0);
  task::sleep(700);

  stopAllIntakes();

  openRatchets();


//Top Right Goal

//back up from TopMid
  driveTo(67, 115, M_PI_2, 650, 1.0);
  waitUntil(runChassisControl == false);

  turnTo(3 * M_PI_4, 500);

  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  task::sleep(400);

  stopBottomRoller();
  stopTopRoller();

  driveTo(114, 120, 0, 1500, 1.0);
  openRatchets();
  task::sleep(1500);
  closeRatchets();

  runIntakes(1.0, 2, 1000);

  waitUntil(runTheIntakes == false);

  driveTo(126, 100, 0, 1000, 1.0);

  task::sleep(200);

  openRatchets();

  task::sleep(800);

  closeRatchets();
  runIntakes(1.0, 3, 2000);

  task::sleep(200);

  driveTo(112, 115, M_PI_4, 800, 1.0);
  waitUntil(runChassisControl == false);

  driveTo(130, 135, M_PI_4, 800, 1.0);
  waitUntil(runChassisControl == false);
//score 2 descore 2
  runAllIntakes(1.0);

  task::sleep(700);

  stopAllIntakes();

  openRatchets();

  task::sleep(300);

  closeRatchets();
  runIntakes(1.0, 3, 700);
 
  waitUntil(runTheIntakes == false);

//Middle Right Goal

//back up from TR

  driveTo(87, 90, M_PI_4, 2000, 1.0);

  task::sleep(200);

  openRatchets();

  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  task::sleep(700);

  stopBottomRoller();
  stopTopRoller();

  driveTo(87, 90, 3 * M_PI_2, 1200, 1.0);
  waitUntil(runChassisControl == false);

  driveTo(87, 70, 3 * M_PI_2, 1000, 1.0);

  task::sleep(750);

  closeRatchets();

  runIntakes(1.0, 2, 2000);

  turnTo(7 * M_PI_4, 400);
  task::sleep(200);
  turnTo(0, 300);

  waitUntil(runChassisControl == false);

  driveTo(125, 70, 0, 1100, 1.0);

  openRatchets();

  task::sleep(900);

  closeRatchets();

  runIntakes(1.0, 3, 2000);

  waitUntil(runChassisControl == false);

  runAllIntakes(1.0);
  task::sleep(700);

  stopAllIntakes();

  openRatchets();

//BOTTOM RIGHT

//back away from mid right
  openRatchets();
  driveTo(93, 69, 0, 700, 1.0);
  waitUntil(runChassisControl == false);

//eject 1 blue ball
  turnTo(M_PI_4, 500);

  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  task::sleep(600);

  stopBottomRoller();
  stopTopRoller();

  closeRatchets();

  driveTo(93, 17, 3 * M_PI_2, 2000, 1.0);
  
  task::sleep(600);

  openRatchets();

  task::sleep(1300);

  closeRatchets();
  runIntakes(1.0, 2, 2000);
  waitUntil(runChassisControl == false);

  driveTo(124, 35, 0, 1500, 1.0);

  task::sleep(400);
  openRatchets();

  waitUntil(runChassisControl == false);

  closeRatchets();
  runIntakes(1.0, 3, 2000);

//line up to BR goal

  driveTo(117, 20, 7 * M_PI_4, 1000, 1.0);
  waitUntil(runChassisControl == false);

//drive into goal

  driveTo(126, 5, 7 * M_PI_4, 1000, 1.0);

  runJustIntakes(1.0, 0, 900);
  waitUntil(runChassisControl == false);

//score 2 descore 2
  runAllIntakes(1.0);

  task::sleep(650);

  stopAllIntakes();

  openRatchets();

  task::sleep(300);

  closeRatchets();
  runIntakes(1.0, 3, 800);
 
  waitUntil(runTheIntakes == false);

//BOTTOM MIDDLE

//back up from BR goal

  driveTo(100, 35, 7 * M_PI_4, 2000, 1.0);
//release 2 blue
  
  openRatchets();
  task::sleep(200);
  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  task::sleep(700);

  stopBottomRoller();
  stopTopRoller();

  driveTo(64, 53, M_PI, 1900, 1.0);

  openRatchets();

  task::sleep(1900);
  closeRatchets();
  runIntakes(1.0, 2, 2000);

//drive up to bottom mid

  driveTo(64, 6, 3 * M_PI_2, 1500, 1.0);
  waitUntil(runChassisControl == false);
  runAllIntakes(1.0);
  task::sleep(700);

  stopAllIntakes();

  //back up
  driveTo(64, 30, 3 * M_PI_2, 1000, 1.0);

  //126!


}




void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  //reset rotation sensors
  LTrack.resetPosition();
  RTrack.resetPosition();
  STrack.resetPosition();

  FrontLeftDrive.resetRotation();
  FrontRightDrive.resetRotation();
  BackLeftDrive.resetRotation();
  BackRightDrive.resetRotation();

  //start the odometry
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);
  task intakeTask(intakeControl);

  autonSkills();
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
double exponentialDrive(double controllerValue) {
  return pow(controllerValue, 3) / pow(100, 2);

}

bool areIntakesOpenInDriver = false;

void openRatchetsInDriver() {
  LeftIntake.resetPosition();
  RightIntake.resetPosition();
}

double leftError = 0;
double rightError = 0;
double intakekPDriver = 1.0;

void usercontrol(void) {

  double driveAmt;
  double turnAmt;
  double strafeAmt;

  FrontLeftDrive.setBrake(brakeType::brake);
  FrontRightDrive.setBrake(brakeType::brake);
  BackLeftDrive.setBrake(brakeType::brake);
  BackRightDrive.setBrake(brakeType::brake);

  while (1) {

    /* DRIVE */
    driveAmt = exponentialDrive(Controller1.Axis3.value());
    turnAmt = 0.7 * exponentialDrive(Controller1.Axis1.value());
    strafeAmt = exponentialDrive(Controller1.Axis4.value());

    FrontLeftDrive.spin(directionType::fwd, driveAmt + turnAmt + strafeAmt, velocityUnits::pct);
    FrontRightDrive.spin(directionType::fwd, driveAmt - turnAmt - strafeAmt, velocityUnits::pct);
    BackLeftDrive.spin(directionType::fwd, driveAmt + turnAmt - strafeAmt, velocityUnits::pct);
    BackRightDrive.spin(directionType::fwd, driveAmt - turnAmt + strafeAmt, velocityUnits::pct);

    if(abs(Controller1.Axis3.value()) < 5 && abs(Controller1.Axis1.value()) < 5 && abs(Controller1.Axis4.value()) < 5) {
      FrontLeftDrive.stop(brakeType::brake);
      FrontRightDrive.stop(brakeType::brake);
      BackLeftDrive.stop(brakeType::brake);
      BackRightDrive.stop(brakeType::brake);
    }

    /* INTAKES AND BOTTOM INDEXER */
    
    //L1 runs the intakes and bottom indexer inward
    if(Controller1.ButtonL1.pressing()) {
      areIntakesOpenInDriver = false;
      LeftIntake.spin(directionType::fwd, 100, velocityUnits::pct);
      RightIntake.spin(directionType::fwd, 100, velocityUnits::pct);
      BottomRoller.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    //L2 runs the intakes and bottom indexer out
    else if(Controller1.ButtonL2.pressing()) {
      areIntakesOpenInDriver = false;
      LeftIntake.spin(directionType::rev, 100, velocityUnits::pct);
      RightIntake.spin(directionType::rev, 100, velocityUnits::pct);

      //Only run the bottom indeder out if we're not running the top indexer up
      if(!Controller1.ButtonR1.pressing()) {
        BottomRoller.spin(directionType::rev, 100, velocityUnits::pct);
      }
      
    }
    //open the ratchets is A is pressed and no other intake commands are happening
    else if(Controller1.ButtonA.pressing() && areIntakesOpenInDriver == false) {
      areIntakesOpenInDriver = true;

    }
    //Else, stop the intakes and bottom indexer
    else {
      LeftIntake.stop(brakeType::brake);
      RightIntake.stop(brakeType::brake);
      //Stop the bottom roller, unless we're scoring balls
      if(!Controller1.ButtonR1.pressing()) {
        BottomRoller.stop(brakeType::brake);
      } 
    }

    /* TOP INDEXER */

    //R1 runs top & bottom indexers upwards
    if(Controller1.ButtonR1.pressing()) {
      TopRoller.spin(directionType::fwd, 100, velocityUnits::pct);
      BottomRoller.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    //R2 runs the top indexer downwards
    else if(Controller1.ButtonR2.pressing()) {
      TopRoller.spin(directionType::rev, 100, velocityUnits::pct);
    }
    //Else, stop the top indexer
    else {
      TopRoller.stop(brakeType::brake);
    }


    //ratchet control
    if(areIntakesOpenInDriver == true) {
      leftError = -45 - LeftIntake.position(rotationUnits::deg);
      rightError = -45 - RightIntake.position(rotationUnits::deg);

      LeftIntake.spin(directionType::fwd, leftError * intakekPDriver, voltageUnits::volt);
      RightIntake.spin(directionType::fwd, rightError * intakekPDriver, voltageUnits::volt);
    }


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
