#ifndef FinalizeLiftAlignment_H
#define FinalizeLiftAlignment_H

#include "../CommandBase.h"

class FinalizeLiftAlignment : public CommandBase {
public:
	FinalizeLiftAlignment();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // FinalizeLiftAlignment_H
