#include "DriveToNearestRope.h"

DriveToNearestRope::DriveToNearestRope() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(drivetrain);
}

// Called just before this Command runs the first time
void DriveToNearestRope::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveToNearestRope::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool DriveToNearestRope::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveToNearestRope::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToNearestRope::Interrupted() {

}
