#ifndef DriveByDistance_H
#define DriveByDistance_H

#include "../CommandBase.h"

class DriveByDistance : public CommandBase {
public:
	DriveByDistance(double by);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double inches;// definitions
	double encoderStart;//even more
	double encoderTarget;
};

#endif  // DriveByDistance_H
