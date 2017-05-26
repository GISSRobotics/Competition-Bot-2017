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
	// Uncomment for better straigtness - needs testing!
	gyroTarget = drivetrain->GetGyroAngle();

}

// Called repeatedly when this Command is scheduled to run
void DriveByDistance::Execute() {
	// TO IMPLEMENT: Uncomment for better straigtness - needs testing!
	if (drivetrain->GetEncoderAverageDistance() < encoderTarget) {
		double ts = 0.30;
		double err = (drivetrain->GetGyroAngle()-gyroTarget)/30.0;
		drivetrain->ManualDrive(ts-err, ts+err);
	} else{
		drivetrain->ManualDrive(0.0, 0.0);
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
