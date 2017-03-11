#include "DriveWithJoystick.h"

DriveWithJoystick::DriveWithJoystick() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(drivetrain.get());
}

// Called just before this Command runs the first time
void DriveWithJoystick::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute() {
	Joystick* stick = oi->GetJoystick();
	double sensitivity = (stick->GetRawAxis(3)-1)/2;
	if (std::abs(stick->GetRawAxis(0))>0.01||std::abs(stick->GetRawAxis(2))>0.01) {
		drivetrain->isInUse = false;
		drivetrain->Drive(stick->GetRawAxis(1)*sensitivity, stick->GetRawAxis(0)*sensitivity);
	} else if (!drivetrain->isInUse) {
		drivetrain->Drive(0.0, 0.0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoystick::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveWithJoystick::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoystick::Interrupted() {

}
