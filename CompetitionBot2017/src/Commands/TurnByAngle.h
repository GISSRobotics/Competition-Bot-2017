#ifndef TurnByAngle_H
#define TurnByAngle_H


#include "../CommandBase.h"

class TurnByAngle : public CommandBase {
public:
	ClimbRope();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif // TurnByAngle_H