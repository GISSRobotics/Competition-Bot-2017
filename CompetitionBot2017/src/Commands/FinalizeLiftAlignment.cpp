#include "FinalizeLiftAlignment.h"
#include "DriveWithJoystick.h"

FinalizeLiftAlignment::FinalizeLiftAlignment() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void FinalizeLiftAlignment::Initialize() {
	drivetrain->controlsSwapped = false;
	drivetrain->isTargetting = true;
}

// Called repeatedly when this Command is scheduled to run
void FinalizeLiftAlignment::Execute() {
	drivetrain->isInUse = true;
	if (drivetrain->targetRange > 10.0) {
		double ts = 0.3;
		double err = drivetrain->targetCenter/200.0;
		drivetrain->ManualDrive(ts+err, ts-err);
	} else {
		drivetrain->ManualDrive(0.0, 0.0);
	}

	/*
	double s = 0.0;// lookie its a definition
	double a = 0.0;// even more, buy one get one free
	if (drivetrain->targetFound) {
		if (drivetrain->targetCenter < -5.0) {
			s = 0.50;// setting the variable
			a = 0.05;//buy one varible set get one free
		} else if (drivetrain->targetCenter > 5.0) {
			s = -0.50;//and a switchy
			a = 0.05;//see we inversed the power
		}
		if (drivetrain->targetRange > 5.0) {
			a += 0.35;
		} else {
			a = 0.0;
		}
		drivetrain->Drive(a, s+0.08);
	}
	*/
}

// Make this return true when this Command no longer needs to run execute()
bool FinalizeLiftAlignment::IsFinished() {
	bool isDone = (drivetrain->targetFound&&drivetrain->targetCenter>=-5.0&&drivetrain->targetCenter<=5.0&&drivetrain->targetRange<=5.0)||(!drivetrain->targetFound);
	frc::SmartDashboard::PutBoolean("Target Aligned", isDone);
	return isDone||!drivetrain->isInUse;
}

// Called once after isFinished returns true
void FinalizeLiftAlignment::End() {
	drivetrain->Drive(0.0, 0.0);
	drivetrain->isInUse = false;
	drivetrain->isTargetting = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FinalizeLiftAlignment::Interrupted() {

}
