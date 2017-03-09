#include <memory>

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "GripPipeline.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <Commands/DriveWithJoystick.h>

#include <Commands/AutonomousCommand.h>

#include "CommandBase.h"

using namespace frc;

class Robot: public frc::IterativeRobot {
public:

	//Setup camera
	static void CVThread() {
		cs::UsbCamera visionCam = CameraServer::GetInstance()->StartAutomaticCapture(0);
		CameraServer::GetInstance()->StartAutomaticCapture(1);
		visionCam.SetResolution(320, 240);
		visionCam.SetExposureManual(20);
		/*cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo(visionCam.GetName());
		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Augmented Display", 640, 480);
		cv::Mat source;
		cv::Mat output;
		while (true) {
			cvSink.GrabFrame(source);
			cv::cvtColor(source, output, cv::COLOR_BGR2GRAY);
			outputStream.PutFrame(output);
		}*/
	}

	//Update camera
	static void VisionThread() {
		cs::UsbCamera visionCam = CameraServer::GetInstance()->StartAutomaticCapture(1);
		CameraServer::GetInstance()->StartAutomaticCapture(0);
		visionCam.SetResolution(320, 240);
		visionCam.SetExposureManual(20);
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo(visionCam.GetName());
		/*
		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("CV Output", 320, 240);
		cv::Mat source;
		cv::Mat output;
		grip::GripPipeline pipeline;
		while (true) {
			cvSink.GrabFrame(source);
			frc::SmartDashboard::PutNumber("scn", source.channels());
			//cv::cvtColor(source, output, cv::COLOR_BGR2GRAY);
			//outputStreamStd.PutFrame(source);

			//pipeline.Process(source);
			//output = pipeline.GetMaskOutput();
			/*std::vector<std::vector<cv::Point>> contours = pipeline.GetFilterContoursOutput();
			frc::SmartDashboard::PutNumber("Contours Found:", contours.size());
			double D = 8.25;
			double Sx = 320;
			double AOV = 64.974;
			if (contours.size() >= 2) {
				CommandBase::drivetrain->targetFound = true;
				cv::Rect lRect = cv::boundingRect(contours[0]);
				cv::Rect rRect = cv::boundingRect(contours[1]);
				cv::rectangle(output, lRect, cv::Scalar(0, 0, 255));
				cv::rectangle(output, rRect, cv::Scalar(255, 0, 0));
				frc::SmartDashboard::PutNumber("Left CV", lRect.x);
				frc::SmartDashboard::PutNumber("Right CV", rRect.x);
				frc::SmartDashboard::PutNumber("Px", std::abs(rRect.x-lRect.x));
				double Px = std::abs(rRect.x-lRect.x);
				double d2r = 2*3.14159/360.0;
				double range = (D/(2.0*std::tan((double)d2r*(AOV*Px/(2.0*Sx)))))-16.0;
				frc::SmartDashboard::PutNumber("Target Range", range);
				CommandBase::drivetrain->targetRange = range;
				double centeredness = ((rRect.x+lRect.x)/2.0)-160.0;
				frc::SmartDashboard::PutNumber("Target Center", centeredness);
				CommandBase::drivetrain->targetCenter = centeredness;
			} else {
				CommandBase::drivetrain->targetFound = false;
			}*/
			//frc::SmartDashboard::PutBoolean("Target Found", CommandBase::drivetrain->targetFound);
			//outputStreamStd.PutFrame(output);
		//}

	}

	//Initializes drive station variables, and sends it to Dashboard
	void RobotInit() override {
		autoChooser.AddDefault("Red Left", new AutonomousCommand(0));
		autoChooser.AddObject("Red Center", new AutonomousCommand(1));
		autoChooser.AddObject("Red Right", new AutonomousCommand(2));
		autoChooser.AddObject("Blue Left", new AutonomousCommand(3));
		autoChooser.AddObject("Blue Center", new AutonomousCommand(4));
		autoChooser.AddObject("Blue Right", new AutonomousCommand(5));
		frc::SmartDashboard::PutData("Drive Station", &autoChooser);
		std::thread visionThread(CVThread);
		visionThread.detach();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {
		//if (CommandBase::drivetrain.get() != nullptr) {
		//	CommandBase::drivetrain->Reset();
		//	CommandBase::gearsleeve->Reset();
		//	CommandBase::winch->Reset();
		//}
	}

	//What happens while disabled
	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	//Begin autonomous
	void AutonomousInit() override {
		autonomousCommand.reset(autoChooser.GetSelected());
		autonomousCommand->Start();
	}

	//Updates during autonomous
	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	//Begin teleop
	void TeleopInit() override {
		//Turn off autonomous
		if (autonomousCommand.get() != nullptr) {
			autonomousCommand->Cancel();
		}
		//Begin driving with joystick
		driveWithJoystick.reset(new DriveWithJoystick());
		driveWithJoystick->Start();
		pdp = new PowerDistributionPanel(0);
	}

	//Updates during teleop
	//Updates gear sleeve and dashboard with new data
	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();

		CommandBase::gearsleeve->Update();

		double volts = pdp->GetVoltage();
		double totalCurrent = pdp->GetTotalCurrent();
		//double current0 = pdp->GetCurrent(0);
		//double current1 = pdp->GetCurrent(1);
		//double current2 = pdp->GetCurrent(2);
		//double current3 = pdp->GetCurrent(3);
		//double current12 = pdp->GetCurrent(12);
		//double totalPower = volts*totalCurrent;

		SmartDashboard::PutNumber("Battery Voltage", volts);
		SmartDashboard::PutNumber("Total Current", totalCurrent);
		//SmartDashboard::PutNumber("TotalPower", totalPower);
		//SmartDashboard::PutNumber("Current 0", current0);
		//SmartDashboard::PutNumber("Current 1", current1);
		//SmartDashboard::PutNumber("Current 2", current2);
		//SmartDashboard::PutNumber("Current 3", current3);
		//SmartDashboard::PutNumber("Current 12", current12);
	}

	//Updates during test mode
	void TestPeriodic() override {
		frc::LiveWindow::GetInstance()->Run();
	}

private:
	std::unique_ptr<frc::Command> autonomousCommand;
	std::unique_ptr<frc::Command> driveWithJoystick;
	PowerDistributionPanel* pdp;
	frc::SendableChooser<frc::Command*> autoChooser;
};

START_ROBOT_CLASS(Robot)
