using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FrontLeftDrive;
extern motor FrontRightDrive;
extern motor BackLeftDrive;
extern motor BackRightDrive;
extern rotation LTrack;
extern rotation RTrack;
extern rotation STrack;
extern motor LeftIntake;
extern motor RightIntake;
extern motor TopRoller;
extern motor BottomRoller;
extern sonar RangeFinderC;
extern sonar RangeFinderE;
extern inertial Inertial6;
extern sonar RangeFinderA;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );