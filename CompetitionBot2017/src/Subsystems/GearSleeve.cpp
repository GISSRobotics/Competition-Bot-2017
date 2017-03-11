#include "GearSleeve.h"
#include "../RobotMap.h"

GearSleeve::GearSleeve() : Subsystem("GearSleeve") {
}

void GearSleeve::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void GearSleeve::Toggle() {
	isUp = !isUp;
	isUpStatus = isUp;
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
}

void GearSleeve::Raise() {
	isUp = true;
	isUpStatus = true;
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
}

bool GearSleeve::Update() {
	if (!definedYet) {
		loadedSwitch = new frc::DigitalInput(GEAR_STATUS_SWITCH);
		gearMotor = new frc::VictorSP(GEAR_MOTOR);
		upLimit = new frc::Counter(GEAR_LIMIT_SWITCH_UP);
		downLimit = new frc::DigitalInput(GEAR_LIMIT_SWITCH_DOWN);
		definedYet = true;
	}
	isLoaded = !loadedSwitch->Get();
	frc::SmartDashboard::PutBoolean("Gear Loaded?", isLoaded);
	if (isUp) {
		isUpStatus = !isUpStatus;
	}
	if (isUp&&upLimit->Get()<1) {
		gearMotor->Set(upPower);
	} else if (!isUp&&downLimit->Get()) {
		gearMotor->Set(downPower);
		upLimit->Reset();
	} else {
		gearMotor->Set(0.0);
	}
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
	frc::SmartDashboard::PutBoolean("Upper Limit", upLimit->Get());
	frc::SmartDashboard::PutBoolean("Lower Limit", downLimit->Get());
	return isLoaded;
}

void GearSleeve::Reset() {
	isUp = false;
	isUpStatus = false;
	isLoaded = false;
	frc::SmartDashboard::PutBoolean("Gear Loaded?", isLoaded);
	frc::SmartDashboard::PutBoolean("Gear Up?", isUpStatus);
}
