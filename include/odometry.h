#include "vex.h"

//The current angle of the bot (RADIANS)
extern double currentAbsoluteOrientation;

//The global position of the bot (INCHES)
extern double xPosGlobal;
extern double yPosGlobal;

//The odometry function
int positionTracking();