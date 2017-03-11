#include "TurnByAngle.h"

TurnByAngle::TurnByAngle(double by) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	degrees = by;
}

// Called just before this Command runs the first time
void TurnByAngle::Initialize() {
	drivetrain->isInUse = true;
	gyroStart = drivetrain->GetGyroAngle();
	gyroTarget = gyroStart + degrees;
}

// Called repeatedly when this Command is scheduled to run
void TurnByAngle::Execute() {
	if ((gyroTarget > gyroStart&&drivetrain->GetGyroAngle() < gyroTarget)||(gyroTarget < gyroStart&&drivetrain->GetGyroAngle() > gyroTarget)) {
		drivetrain->controlsSwapped = false;
		if (degrees > 0) {
			drivetrain->Drive(0.0, -0.52);
		} else {
			drivetrain->Drive(0.0, 0.52);
		}
	} else {
		drivetrain->Drive(0.0, 0.0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool TurnByAngle::IsFinished() {
	return (gyroTarget > gyroStart&&drivetrain->GetGyroAngle() >= gyroTarget)||(gyroTarget < gyroStart&&drivetrain->GetGyroAngle() <= gyroTarget)||!drivetrain->isInUse;
}

// Called once after isFinished returns true
void TurnByAngle::End() {
	drivetrain->Drive(0.0, 0.0);
	drivetrain->isInUse = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnByAngle::Interrupted() {

}
