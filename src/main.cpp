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
      if(intakeStopCondition == 1 && RangeFinderC.distance(distanceUnits::in) < 4) {
        runTheIntakes = false;
        runTheBottomRoller = false;
      }
      else if(intakeStopCondition == 2 && RangeFinderE.distance(distanceUnits::in) < 4) {
        runTheIntakes = false;
        runTheBottomRoller = false;
      }
      else if(intakeStopCondition == 3 && RangeFinderC.distance(distanceUnits::in) < 4 && RangeFinderE.distance(distanceUnits::in) < 4) {
        runTheIntakes = false;
        runTheBottomRoller = false;
      }


      //timeout (only for when there is a stop condition)
      if(totalTimeSpent > maxAllowedTime && intakeStopCondition != 0) {
        runTheIntakes = false;
        runTheBottomRoller = false;
      }

      totalTimeSpent += 20;

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

void runBottomRoller(double speed) {
  runTheBottomRoller = true;

  bottomRollerSpeed = speed;
}

void runTopRoller(double speed) {
  runTheTopRoller = true;
  topRollerSpeed = speed;
}

void runAllIntakes(double speed, int timeOutTime = 1000) {
  totalTimeSpent = 0;
  runTheIntakes = true;
  runTheBottomRoller = true;
  runTheTopRoller = true;
  intakeSpeed = speed;
  bottomRollerSpeed = speed;
  topRollerSpeed = speed;
  intakeStopCondition = 0;
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

  wait(450, timeUnits::msec);
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

  //driveTo(42.8, 9, 0, 5000, 1.0);

  //task::sleep(500000);

  //drive

//BOTTOM LEFT

  //deploy hood (score BL goal)
  runTheBottomRoller = true;
  task::sleep(400);
  runTheBottomRoller = false;

  driveTo(55, 18, M_PI, 1000, 1.0);

  waitUntil(runChassisControl == false);

  turnToPoint(3.15, 35.5, 1000);

  //turnTo(3 * M_PI_4, 1000);

  waitUntil(runChassisControl == false);

  driveTo(10, 35, currentAbsoluteOrientation, 2000, 1.0);

  openRatchets();
  task::sleep(500);
  closeRatchets();

  runIntakes(1.0, 1, 1000);

  waitUntil(runTheIntakes == false);

  openRatchets();
  task::sleep(700);
  closeRatchets();

  runIntakes(1.0, 3, 1000);

//line up to BL goal
  driveTo(20, 20, 5 * M_PI_4, 1000, 1.0);
  waitUntil(runChassisControl == false);
//drive up to BL goal and score
  driveTo(10, 10, 5 * M_PI_4, 1000, 1.0);
  waitUntil(runChassisControl == false);

  runAllIntakes(1.0, 2000);
  task::sleep(600);

  waitUntil(RangeFinderE.distance(distanceUnits::in) < 4 && RangeFinderE.distance(distanceUnits::in) > 0.1);

  runTheTopRoller = false;
  task::sleep(400);

  waitUntil(RangeFinderC.distance(distanceUnits::in) < 4 && RangeFinderC.distance(distanceUnits::in) > 0.1);

  stopAllIntakes();

//back up from BL goal
  driveTo(24, 40, 5 * M_PI_4, 1200, 1.0);
  openRatchets(false);
  task::sleep(200);

//release balls

  runBottomRoller(-1.0);
  runTopRoller(-1.0);

  waitUntil(runChassisControl == false);

  task::sleep(300);

  stopBottomRoller();
  stopTopRoller();

  ratchetIsOpen = false;

//turn toward next ball
  turnTo(M_PI_2, 1000);

  waitUntil(runChassisControl == false);

  driveTo(24, 71, M_PI_2, 2000, 1.0);

  task::sleep(700);

  openRatchets();
  task::sleep(500);
  closeRatchets();

  runIntakes(1.0, 2, 1500);
//hi
  turnTo(M_PI, 1000);
  waitUntil(runChassisControl == false);

  driveTo(9, 71, M_PI, 700, 1.0);
  waitUntil(runChassisControl == false);
//score mid Right goal
  runAllIntakes(1.0);
  task::sleep(400);
//wait for a ball in the bottom area
  waitUntil(RangeFinderC.distance(distanceUnits::in) < 4 && RangeFinderC.distance(distanceUnits::in) > 0.1);

  stopAllIntakes();

  task::sleep(200);

  openRatchets(false);

  driveTo(24, 71, M_PI, 700, 1.0);
  waitUntil(runChassisControl == false);
  turnTo(3 * M_PI_2, 700);
  runBottomRoller(-1.0);
  runTopRoller(-1.0);
  task::sleep(600);
  stopBottomRoller();
  stopTopRoller();
  ratchetIsOpen = false;

//turn toward middle goal
  turnTo(0, 700);

  waitUntil(runChassisControl == false);

  driveTo(73, 70, 0, 2000, 0.7);

  openRatchets();
  task::sleep(500);
  closeRatchets();

  runIntakes(1.0, 2, 1500);

  task::sleep(400);

  //open ratchets to wrap middle goal
  openRatchets(false);

  waitUntil(runChassisControl == false);
  //close ratchets to wrap middle goal
  closeRatchets();

  runAllIntakes(1.0);

  task::sleep(300);

  waitUntil(RangeFinderE.distance(distanceUnits::in) < 4 && RangeFinderE.distance(distanceUnits::in) > 0.1);

  runTheTopRoller = false;
  task::sleep(200);

  waitUntil(RangeFinderC.distance(distanceUnits::in) < 4 && RangeFinderC.distance(distanceUnits::in) > 0.1);

  stopAllIntakes();
  







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
      LeftIntake.spin(directionType::fwd, 100, velocityUnits::pct);
      RightIntake.spin(directionType::fwd, 100, velocityUnits::pct);
      BottomRoller.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    //L2 runs the intakes and bottom indexer out
    else if(Controller1.ButtonL2.pressing()) {
      LeftIntake.spin(directionType::rev, 100, velocityUnits::pct);
      RightIntake.spin(directionType::rev, 100, velocityUnits::pct);

      //Only run the bottom indeder out if we're not running the top indexer up
      if(!Controller1.ButtonR1.pressing()) {
        BottomRoller.spin(directionType::rev, 100, velocityUnits::pct);
      }
      
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
