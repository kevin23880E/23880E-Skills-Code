#include "chassis-control.h"

//target coords to drive to
double xTargetLocation = xPosGlobal;
double yTargetLocation = yPosGlobal;
double targetFacingAngle = 0;

//distances between robot's current position and the target position
double xDistToTarget = 0;
double yDistToTarget = 0;

//angle of hypotenuse of X and Y distances
double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

//front left + back right drive power
double drivePowerFLBR = 0;
//front right + back left drive power
double drivePowerFRBL = 0;

bool runChassisControl = false;

int timeOutValue = 2500;

double maxAllowedSpeed = 1.0;

//Sets the target position and indicates a specific target heading
void driveTo(double xTarget, double yTarget, double targetAngle, double timeOutLength = 2500, double maxSpeed = 1.0) {
  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  targetFacingAngle = targetAngle;
  runChassisControl = true;
  timeOutValue = timeOutLength;
  Brain.resetTimer();
  maxAllowedSpeed = maxSpeed;
}


// turns toward a specific heading
void turnTo(double targetAngle, double timeOutLength = 2500) {
  targetFacingAngle = targetAngle;

  xTargetLocation = xPosGlobal;
  yTargetLocation = yPosGlobal;

  runChassisControl = true;

  timeOutValue = timeOutLength;


  Brain.resetTimer();
}

void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength = 1500) {
  targetFacingAngle = atan2(yCoordToFace - yPosGlobal, xCoordToFace - xPosGlobal);

  if(targetFacingAngle < 0) {
    targetFacingAngle = 2 * M_PI - fabs(targetFacingAngle);
  }
  xTargetLocation = xPosGlobal;
  yTargetLocation = yPosGlobal;

  runChassisControl = true;

  timeOutValue = timeOutLength;


  Brain.resetTimer();
}

/* 
  Sets the drive power for each set of opposing corners. 
  input: The angle of the target relative to the robot's "forward"
  result: Sets each value to a decimal from 0.0 to 1.0 representing 0% to 100% motor power
*/
void setDrivePower(double theta) {
  drivePowerFLBR = sin(theta + M_PI_4) / sin(M_PI_4);

  //Limits the value to 1
  if(fabs(drivePowerFLBR) > 1) {
    drivePowerFLBR = fabs(drivePowerFLBR) / drivePowerFLBR;
  }

  drivePowerFRBL = sin(theta - M_PI_4) / sin(M_PI_4);

  //Limits the value to 1
  if(fabs(drivePowerFRBL) > 1) {
    drivePowerFRBL = fabs(drivePowerFRBL) / drivePowerFRBL;
  }

}

double driveError = 0;
double drivePrevError = 0;

double driveMaxError = 0.1;

double driveIntegral = 0;
double driveIntegralBound = 1.5;

double driveDerivative = 0;

double drivekP = 1.5;

double drivekI = 0.02;
double drivekD = 10.0;

// double drivekP = 0.55;

// double drivekI = 0.05;
// double drivekD = 0.42;

double drivePowerPID = 0;

void drivePID() {

  //Error is equal to the total distance away from the target (uses distance formula with current position and target location)
  driveError = sqrt(pow((xPosGlobal - xTargetLocation), 2) + pow((yPosGlobal - yTargetLocation), 2));
  
  //only use integral if close enough to target
  if(fabs(driveError) < driveIntegralBound) {
    driveIntegral += driveError;
  }
  else {
    driveIntegral = 0;
  }

  //reset integral if we pass the target
  if(driveError * drivePrevError < 0) {
    driveIntegral = 0;
  } 

  driveDerivative = driveError - drivePrevError;

  drivePrevError = driveError;

  drivePowerPID = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

  //Limit power output to 12V
  if(drivePowerPID > 12) {
    drivePowerPID = 12;
  }

  if(fabs(driveError) < driveMaxError) {
    drivePowerPID = 0;
  }

}

double turnError = 0;
double turnPrevError = 0;

double turnMaxError = 0.01;

double turnIntegral = 0;
double turnIntegralBound = 0.09;

double turnDerivative = 0;

double turnkP = 13.00;
double turnkI = 1.00;
double turnkD = 10.00;

double turnPowerPID = 0;

void turnPID() {

  //Error is equal to the difference between the current facing direction and the target direction
  turnError = currentAbsoluteOrientation - targetFacingAngle;

  if(fabs(turnError) > M_PI) {
    turnError = (turnError/fabs(turnError)) * -1 * fabs(2 * M_PI - turnError);
  }

  //only use integral if close enough to target
  if(fabs(turnError) < turnIntegralBound) {
    turnIntegral += turnError;
  }
  else {
    turnIntegral = 0;
  }

  //reset integral if we pass the target
  if(turnError * turnPrevError < 0) {
    turnIntegral = 0;
  } 

  turnDerivative = turnError - turnPrevError;

  turnPrevError = turnError;

  turnPowerPID = (turnError * turnkP + turnIntegral * turnkI + turnDerivative * turnkD);

  //Limit power output to 12V
  if(turnPowerPID > 12) {
    turnPowerPID = 12;
  }

  if(fabs(turnError) < turnMaxError) {
    turnPowerPID = 0;
  }

}

/*
NOTES

Bottom left corner of field is (0,0)

Angles are like a unit circle, so the positive X direction is 0 and positive Y direction is pi/2

*/

double FrontLeftPower = 0;
double FrontRightPower = 0;
double BackLeftPower = 0;
double BackRightPower = 0;

/* CHASSIS CONTROL TASK */
int chassisControl() {

  //loop to constantly execute chassis commands
  while(1) {
    
    if(runChassisControl) {
      //Distances to target on each axis
      xDistToTarget = xTargetLocation - xPosGlobal;
      yDistToTarget = yTargetLocation - yPosGlobal;

      //Angle of hypotenuse
      hypotenuseAngle = atan2(yDistToTarget, xDistToTarget);

      if(hypotenuseAngle < 0) {
        hypotenuseAngle += 2 * M_PI;
      }

      //The angle the robot needs to travel relative to its forward direction in order to go toward the target
      robotRelativeAngle = hypotenuseAngle - currentAbsoluteOrientation + M_PI_2;

      if(robotRelativeAngle > 2 * M_PI) {
        robotRelativeAngle -= 2 * M_PI;
      }
      else if(robotRelativeAngle < 0) {
        robotRelativeAngle += 2 * M_PI;
      }

      //Get the power percentage values for each set of motors
      setDrivePower(robotRelativeAngle);

      //get PID values for driving and turning
      drivePID();
      turnPID();

      //set power for each motor
      FrontLeftPower = (turnPowerPID + (drivePowerFLBR * drivePowerPID)) * maxAllowedSpeed;
      FrontRightPower = ((drivePowerFRBL * drivePowerPID) - turnPowerPID) * maxAllowedSpeed;
      BackLeftPower = ((drivePowerFRBL * drivePowerPID) + turnPowerPID) * maxAllowedSpeed;
      BackRightPower = ((drivePowerFLBR * drivePowerPID) - turnPowerPID) * maxAllowedSpeed;
      
      FrontLeftDrive.spin(directionType::fwd, FrontLeftPower, voltageUnits::volt);
      FrontRightDrive.spin(directionType::fwd, FrontRightPower, voltageUnits::volt);
      BackLeftDrive.spin(directionType::fwd, BackLeftPower, voltageUnits::volt);
      BackRightDrive.spin(directionType::fwd, BackRightPower, voltageUnits::volt);
      

   
      if(driveError < 0.1 && turnError < 0.003) {
        runChassisControl = false;
      }

      if(Brain.timer(timeUnits::msec) > timeOutValue) {
        runChassisControl = false;
      }

      // Brain.Screen.setCursor(1,2);
      // Brain.Screen.print("Hyp angle: %f", hypotenuseAngle);
      // Brain.Screen.setCursor(2,2);
      // Brain.Screen.print(FrontLeftPower);
      //Brain.Screen.print("botRelativeAngle: %f", robotRelativeAngle);


      Brain.Screen.setCursor(3,2);
      Brain.Screen.print("drive error: %f", driveError);

      Brain.Screen.setCursor(5,2);
      Brain.Screen.print("turn error: %f", turnError);

      Brain.Screen.setCursor(7,2);
      Brain.Screen.print("turnPID power: %f", drivePowerPID);

      Brain.Screen.setCursor(8,2);
      Brain.Screen.print("FL Power: %f", FrontLeftPower);

      // Brain.Screen.setCursor(4,2);
      // Brain.Screen.print("absolute orientation: %f", currentAbsoluteOrientation);
      // // Brain.Screen.setCursor(3,2);
      // Brain.Screen.print(drivePowerFLBR);

    }
    //What to do when not using the chassis controls
    else {
      // FrontLeftDrive.stop(brakeType::brake);
      // FrontRightDrive.stop(brakeType::brake);
      // BackLeftDrive.stop(brakeType::brake);
      // BackRightDrive.stop(brakeType::brake);
      FrontLeftDrive.stop(brakeType::coast);
      FrontRightDrive.stop(brakeType::coast);
      BackLeftDrive.stop(brakeType::coast);
      BackRightDrive.stop(brakeType::coast);
    }
    
    task::sleep(20);

  }

  return 1;
}