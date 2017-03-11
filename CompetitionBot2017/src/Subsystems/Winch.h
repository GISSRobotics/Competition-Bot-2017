#ifndef Winch_H
#define Winch_H

#include <Commands/Subsystem.h>

#include <WPILib.h>

class Winch : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	VictorSP* winchMotor;
	bool definedYet = false;
public:
	Winch();
	void InitDefaultCommand();
	void SetMotorPower(double power);
	void Reset();
};

#endif  // Winch_H
