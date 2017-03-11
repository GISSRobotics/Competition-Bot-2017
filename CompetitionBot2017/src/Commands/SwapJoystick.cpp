#include "SwapJoystick.h"

SwapJoystick::SwapJoystick() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void SwapJoystick::Initialize() {
	CommandBase::drivetrain->SwapControls();
}

// Called repeatedly when this Command is scheduled to run
void SwapJoystick::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool SwapJoystick::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void SwapJoystick::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SwapJoystick::Interrupted() {

}
