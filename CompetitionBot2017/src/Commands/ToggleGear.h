#ifndef ToggleGear_H
#define ToggleGear_H

#include "../CommandBase.h"

class ToggleGear : public CommandBase {
public:
	ToggleGear();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ToggleGear_H
