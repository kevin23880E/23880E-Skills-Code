#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FrontLeftDrive = motor(PORT11, ratio18_1, false);
motor FrontRightDrive = motor(PORT3, ratio18_1, true);
motor BackLeftDrive = motor(PORT18, ratio18_1, false);
motor BackRightDrive = motor(PORT5, ratio18_1, true);
rotation LTrack = rotation(PORT12, true);
rotation RTrack = rotation(PORT17, false);
rotation STrack = rotation(PORT20, true);
motor LeftIntake = motor(PORT1, ratio18_1, false);
motor RightIntake = motor(PORT8, ratio18_1, true);
motor TopRoller = motor(PORT9, ratio6_1, false);
motor BottomRoller = motor(PORT10, ratio6_1, false);
sonar RangeFinderC = sonar(Brain.ThreeWirePort.C);
sonar RangeFinderE = sonar(Brain.ThreeWirePort.E);
inertial Inertial6 = inertial(PORT6);
sonar RangeFinderA = sonar(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}