// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/Drivebase.h"
#include "subsystems/Lift.h"
#include "subsystems/Climb.h"
#include "subsystems/Intake.h"

class RobotContainer
{
public:
	RobotContainer();

	frc2::CommandPtr GetAutonomousCommand();

	// Put additional test commands on the dashboard
	void SetupTestCommands();
	void Enable();
	void Disable();

private:
	// Set up controller bindings to control parts of the robot
	void ConfigureBindings();

	frc2::CommandXboxController m_driverController, m_liftController;

	// Sybsystems
	Drivebase m_drivebase;
	Intake m_intake;
	Lift m_lift;
	Climb m_climb;

	enum class AutoSequence
	{
		Mobility, TroughBump, CoralScoreHigh, CoralScoreLow
	};

	// Dashboard selector for initial drive time
	frc::SendableChooser<units::second_t> m_autoDriveTimeChooser;

	// Dashboard selector object for auto routines
	frc::SendableChooser<AutoSequence> m_autoChooser;
	
	frc2::CommandPtr m_liftDisableCmd;

	// For automatic height testing
	const std::string m_targetHeightKey = "Target Height";
	float m_targetHeight = 0;
};
