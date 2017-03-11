#ifndef AutonomousCommand_H
#define AutonomousCommand_H

#include <Commands/CommandGroup.h>

class AutonomousCommand : public CommandGroup {
public:
	AutonomousCommand(int id);
	const int REDL = 0;// these are things that do stuff
	const int REDC = 1;//all joking aside we define stuff here to use later  
	const int REDR = 2;
	const int BLUEL = 3;
	const int BLUEC = 4;
	const int BLUER = 5;
};

#endif  // AutonomousCommand_H
