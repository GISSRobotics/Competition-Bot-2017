#ifndef OI_H
#define OI_H

#include "WPILib.h"

class OI {
private:
	Joystick* stick;
	JoystickButton* btn1;
	JoystickButton* btn12;
	JoystickButton* btn11;
	JoystickButton* btn9;
	JoystickButton* btn10;

public:
	OI();
	Joystick* GetJoystick();
};

#endif  // OI_H
