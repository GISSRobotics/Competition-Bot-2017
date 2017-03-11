#ifndef DriveTrain_H
#define DriveTrain_H

#include <Commands/Subsystem.h>

#include <WPILib.h>

class DriveTrain : public frc::Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	bool definedYet = false;
	const double pi = 3.14159265358979323846;
	frc::RobotDrive* driveBase;
	frc::Encoder* leftEncoder;
	frc::Encoder* rightEncoder;
	frc::ADXRS450_Gyro* gyro;
public:
	bool controlsSwapped = false;
	bool isInUse = false;
	bool targetFound = false;
	double targetRange = 0.0;
	double targetCenter = 0.0;
	DriveTrain();
	void InitDefaultCommand();
	void Drive(double acceleration, double steering);
	void SwapControls();
	double GetEncoderAverageDistance();
	double GetGyroAngle();
	void Reset();
};

#endif  // DriveTrain_H
