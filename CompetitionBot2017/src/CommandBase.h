#ifndef COMMAND_BASE_H 
#define COMMAND_BASE_H //we define the title of the code to be used in other codes outside this sheet (in this case we define the name as COMMAND_BASE_H)

#include <memory> //we tell the program to remember this programming even if we are in a different code
#include <string>

#include <Commands/Command.h> //We include a program that within it's code it's gonna have something that affects this code as well

#include "OI.h"//same as above for these commands right here
#include "Subsystems/Winch.h"
#include "Subsystems/GearSleeve.h"
#include "Subsystems/DriveTrain.h"

/**
 * The base for all commands. All atomic commands should subclass CommandBase.
 * CommandBase stores creates and stores each control system. To access a
 * subsystem elsewhere in your code in your code use
 * CommandBase::exampleSubsystem
 */
class CommandBase: public frc::Command {
public:
	CommandBase(const std::string& name);
	CommandBase() = default;

	// Create a single static instance of all of your subsystems
	static std::unique_ptr<DriveTrain> drivetrain;
	static std::unique_ptr<GearSleeve> gearsleeve;
	static std::unique_ptr<Winch> winch;
	static std::unique_ptr<OI> oi;
};

#endif  // COMMAND_BASE_H
