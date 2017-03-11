#ifndef GearSleeve_H
#define GearSleeve_H

#include <Commands/Subsystem.h>

#include <WPILib.h>

class GearSleeve : public frc::Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	bool definedYet = false;
	bool isUp = false;
	bool isUpStatus = false;
	bool isLoaded = false;
	//double downPoint = 0.8;
	//double upPoint = 0.2;
	double upPower = 0.7;
	double downPower = -0.7;
	frc::VictorSP* gearMotor;
	frc::DigitalInput* loadedSwitch;
	frc::Counter* upLimit;
	frc::DigitalInput* downLimit;
public:
	GearSleeve();
	void InitDefaultCommand();
	void Toggle();
	void Raise();
	bool Update();
	void Reset();
};

#endif  // GearSleeve_H
