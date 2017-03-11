#include "ToggleGear.h"

ToggleGear::ToggleGear() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(gearsleeve.get());
}

// Called just before this Command runs the first time
void ToggleGear::Initialize() {
	gearsleeve->Toggle();
}

// Called repeatedly when this Command is scheduled to run
void ToggleGear::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool ToggleGear::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void ToggleGear::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ToggleGear::Interrupted() {

}
