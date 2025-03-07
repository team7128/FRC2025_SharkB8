// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/Drivebase.h"
#include "subsystems/Lift.h"
#include "subsystems/Climb.h"
#include "subsystems/CoralAligner.h"

class RobotContainer
{
public:
	RobotContainer();

	frc2::CommandPtr GetAutonomousCommand();

	void SetupTestCommands();
	void Enable();
	void Disable();

//private:
	void ConfigureBindings();

	frc2::CommandXboxController m_driverController, m_liftController;

	Drivebase m_drivebase;
	Lift m_lift;
	Climb m_climb;
	CoralAligner m_aligner;
	frc2::CommandPtr m_liftDisableCmd;

	const std::string m_targetHeightKey = "Target Height";
	float m_targetHeight = 0;
};
