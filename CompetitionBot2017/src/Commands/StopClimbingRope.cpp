#include "StopClimbingRope.h"

StopClimbingRope::StopClimbingRope() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(winch.get());
}

// Called just before this Command runs the first time
void StopClimbingRope::Initialize() {
	winch->SetMotorPower(0.0);
}

// Called repeatedly when this Command is scheduled to run
void StopClimbingRope::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool StopClimbingRope::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void StopClimbingRope::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StopClimbingRope::Interrupted() {

}
