#include "CommandBase.h" //we tell the program that in this code we are gonna use the data from another document

#include <Commands/Scheduler.h> // we show the location of the document where the commands exists already

#include "Subsystems/DriveTrain.h"
#include "Subsystems/GearSleeve.h"
#include "Subsystems/Winch.h"

// Initialize a single static instance of all of your subsystems. The following
// line should be repeated for each subsystem in the project.
std::unique_ptr<DriveTrain> CommandBase::drivetrain = std::make_unique<DriveTrain>();
std::unique_ptr<GearSleeve> CommandBase::gearsleeve = std::make_unique<GearSleeve>();
std::unique_ptr<Winch> CommandBase::winch = std::make_unique<Winch>();

std::unique_ptr<OI> CommandBase::oi = std::make_unique<OI>();

CommandBase::CommandBase(const std::string &name) :
		frc::Command(name) {

}
