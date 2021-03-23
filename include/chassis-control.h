#include "vex.h"
#include "odometry.h"

extern double xTargetLocation;
extern double yTargetLocation;
extern double targetFacingAngle;

extern bool runChassisControl;

extern void driveTo(double xTarget, double yTarget, double targetAngle, double timeOutLength, double maxSpeed);

extern void turnTo(double targetAngle, double timeOutLength);

extern void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength);

void setDrivePower(double theta);

void drivePID();
void turnPID();

int chassisControl();
