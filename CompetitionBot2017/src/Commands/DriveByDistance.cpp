#include "DriveByDistance.h"

DriveByDistance::DriveByDistance(double by) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	inches = by;
}

// Called just before this Command runs the first time
void DriveByDistance::Initialize() {
	drivetrain->isInUse = true;//it is using the drive train
	encoderStart = drivetrain->GetEncoderAverageDistance();//start using the encoders to get the avarge distance
	encoderTarget = encoderStart + inches;// mesurin that distance in inches
}

// Called repeatedly when this Command is scheduled to run
void DriveByDistance::Execute() {
	if (drivetrain->GetEncoderAverageDistance() < encoderTarget) {//starts using the encoder to go the target distance
		drivetrain->controlsSwapped = false;// the controls in drivetrain are not inverted
		if (inches > 0) {// 
			drivetrain->Drive(0.5, 0.2);//sets the speed 
		} else {
			drivetrain->Drive(-0.5, 0.0);//same thing
		}
	} else {
		drivetrain->Drive(0.0, 0.0);//once again same thing
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveByDistance::IsFinished() {
	return drivetrain->GetEncoderAverageDistance() >= encoderTarget||!drivetrain->isInUse;
}

// Called once after isFinished returns true
void DriveByDistance::End() {
	drivetrain->Drive(0.0, 0.0);//sets the spped to 0
	drivetrain->isInUse = false; //kills drive train
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveByDistance::Interrupted() {

}
