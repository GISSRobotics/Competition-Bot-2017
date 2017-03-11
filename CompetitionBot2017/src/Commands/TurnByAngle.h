#ifndef TurnByAngle_H
#define TurnByAngle_H

#include "../CommandBase.h"

class TurnByAngle : public CommandBase {
public:
	TurnByAngle(double by);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double degrees;
	double gyroStart;
	double gyroTarget;
};

#endif  // TurnByAngle_H
