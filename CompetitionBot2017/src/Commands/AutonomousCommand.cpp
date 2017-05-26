#include "AutonomousCommand.h"// I really shouldn't have to comment include statements

#include "DriveByDistance.h"//but these are important
#include "TurnByAngle.h"
#include "FullAutoPlaceGear.h"

AutonomousCommand::AutonomousCommand(int id) {
	// Add Commands here:
	// e.g. AddSequential(new Command1());
	//      AddSequential(new Command2());
	// these will run in order.

	// To run multiple commands at the same time,
	// use AddParallel()
	// e.g. AddParallel(new Command1());
	//      AddSequential(new Command2());
	// Command1 and Command2 will run in parallel.

	// A command group will require all of the subsystems that each member
	// would require.
	// e.g. if Command1 requires chassis, and Command2 requires arm,
	// a CommandGroup containing them would require both the chassis and the
	// arm.

	double dist = 76.0;

	if (id == 2) {
		dist = 68.0;
	}

	AddSequential(new DriveByDistance(dist));

	if (id == 1) {
		AddSequential(new TurnByAngle(48.0));
	} else if (id == 3) {
		AddSequential(new TurnByAngle(-48.0));
	}

	AddSequential(new FullAutoPlaceGear());
}
