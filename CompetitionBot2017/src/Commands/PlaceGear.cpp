#include "PlaceGear.h"

PlaceGear::PlaceGear() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(gearsleeve.get());
}

// Called just before this Command runs the first time
void PlaceGear::Initialize() {
	drivetrain->isTargetting = true;
	if (drivetrain->targetFound&&drivetrain->targetRange<5.0) {
		drivetrain->isInUse = true;
		gearsleeve->Raise();
	}
}

// Called repeatedly when this Command is scheduled to run
void PlaceGear::Execute() {
	gearsleeve->Update();
	if (drivetrain->targetRange < 0.5) {
		drivetrain->ManualDrive(0.0, 0.0);
	} else {
		drivetrain->ManualDrive(0.25, 0.3);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool PlaceGear::IsFinished() {
	return drivetrain->targetRange < 0.5||!drivetrain->targetFound||!drivetrain->isInUse;
}

// Called once after isFinished returns true
void PlaceGear::End() {
	drivetrain->isTargetting = false;
	drivetrain->isInUse = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PlaceGear::Interrupted() {

}
