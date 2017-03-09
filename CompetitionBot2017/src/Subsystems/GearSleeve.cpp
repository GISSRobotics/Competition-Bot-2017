#include "GearSleeve.h"
#include "../RobotMap.h"

GearSleeve::GearSleeve() : Subsystem("GearSleeve") {
}

void GearSleeve::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}
//Toggle current gear sleeve state and update Dashboard
void GearSleeve::Toggle() {
	isUp = !isUp;
	isUpStatus = isUp;
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
}
//Raise gear sleeve and update Dashboard
void GearSleeve::Raise() {
	isUp = true;
	isUpStatus = true;
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
}

bool GearSleeve::Update() {
	//If not defined, define variables
	if (!definedYet) {
		loadedSwitch = new frc::DigitalInput(GEAR_STATUS_SWITCH);
		gearMotor = new frc::VictorSP(GEAR_MOTOR);
		upLimit = new frc::Counter(GEAR_LIMIT_SWITCH_UP);
		downLimit = new frc::DigitalInput(GEAR_LIMIT_SWITCH_DOWN);
		definedYet = true;
	}
	//Updates variables and dashboard
	isLoaded = !loadedSwitch->Get();
	frc::SmartDashboard::PutBoolean("Gear Loaded?", isLoaded);
	if (isUp) {
		isUpStatus = !isUpStatus;
	}
	//Update gear motor, and if too far, shutdown
	if (isUp&&upLimit->Get()<1) {
		gearMotor->Set(upPower);
	} else if (!isUp&&downLimit->Get()) {
		gearMotor->Set(downPower);
		upLimit->Reset();
	} else {
		gearMotor->Set(0.0);
	}
	//Update dashboard
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
	frc::SmartDashboard::PutBoolean("Upper Limit", upLimit->Get());
	frc::SmartDashboard::PutBoolean("Lower Limit", downLimit->Get());
	return isLoaded;
}
//Reset bools and dashboard
void GearSleeve::Reset() {
	isUp = false;
	isUpStatus = false;
	isLoaded = false;
	frc::SmartDashboard::PutBoolean("Gear Loaded?", isLoaded);
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
}
