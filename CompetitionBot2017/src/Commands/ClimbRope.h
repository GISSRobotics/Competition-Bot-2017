#ifndef ClimbRope_H
#define ClimbRope_H

#include "../CommandBase.h"

class ClimbRope : public CommandBase {
public:
	ClimbRope();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ClimbRope_H
