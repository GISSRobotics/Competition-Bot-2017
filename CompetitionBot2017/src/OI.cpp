#include "OI.h"

#include "Commands/ToggleGear.h"
#include "Commands/ClimbRope.h"
#include "Commands/StopClimbingRope.h"
#include "Commands/SwapJoystick.h"
#include "Commands/FullAutoPlaceGear.h"
#include "Commands/AutonomousCommand.h"

OI::OI() {
	stick = new frc::Joystick(0);
	btn1 = new frc::JoystickButton(stick, 1);   // Toggle Gear Sleeve
	btn12 = new frc::JoystickButton(stick, 12); // P: Climb @ 100%. R: "Stop" Climb @ 0%
	btn11 = new frc::JoystickButton(stick, 11); // Swap joystick controls
	btn9 = new frc::JoystickButton(stick, 9);   // Auto place gear
	btn10 = new frc::JoystickButton(stick, 10); // Testing
	btn1->WhenPressed(new ToggleGear());
	btn12->WhenPressed(new ClimbRope());
	btn12->WhenReleased(new StopClimbingRope());
	btn11->WhenPressed(new SwapJoystick());
	btn9->WhenPressed(new FullAutoPlaceGear());
	//btn10->WhenPressed(new AutonomousCommand(0));
}

Joystick* OI::GetJoystick() {
	return stick;
}
