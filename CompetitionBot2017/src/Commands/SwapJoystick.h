#ifndef SwapJoystick_H
#define SwapJoystick_H

#include "../CommandBase.h"

class SwapJoystick : public CommandBase {
public:
	SwapJoystick();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // SwapJoystick_H
