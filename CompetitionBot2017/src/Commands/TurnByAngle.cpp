#include "TurnByAngle.h"

TurnByAngle::TurnByAngle() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());\
	
	/////////////////////////////////////////
	//WHATEVER REQUIRES FOR GYRO AND MOTORS//
	//ALSO REQUIRES 2 PARAMETERS: long val, bool deg (val for input value, deg for determining degree or radians)

}

// Called just before this Command runs the first time
void TurnByAngle::Initialize() {
	public long pi = 3.141592653589793;
	public bool positive = false;//Determines left(negative) or right(positive)
	public bool dirChecked = false//Determines if direction has been checked
	public long currentDirection = 0;//This will be updated by the gyro after each movement
	
	if (!deg){//if input is in radians
		long tempval = val;
		val = (tempval *pi)/180;//Change to degrees
	}
	
	if (val < 0){//if left
		dirChecked = true;//"returns" negative
	}else if (val > 0){//if right
		dirChecked = true;
		positive = true;//returns "positive"
	}else if(val == 0){
		return void;//else stop program
	}
}

// Called repeatedly when this Command is scheduled to run
void TurnByAngle::Execute() {

	if (dirChecked && positive){
		while (currentDir < val){
			//TURN RIGHT A BIT
			currentDir = //Update Gyro
		}
	}
	if (dirChecked && !positive){
		while (currentDir > val){
			//TURN LEFT A BIT
			currentDir = //Update Gyro
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool TurnByAngle::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void TurnByAngle::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnByAngle::Interrupted() {
	//When interupted, subtract current angle by goal angle, restart after a time or cache the value to be used again
}