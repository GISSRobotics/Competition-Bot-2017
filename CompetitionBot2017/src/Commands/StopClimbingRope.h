#ifndef StopClimbingRope_H
#define StopClimbingRope_H

#include "../CommandBase.h"

class StopClimbingRope : public CommandBase {
public:
	StopClimbingRope();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // StopClimbingRope_H
