// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <cmath>

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"

RobotContainer::RobotContainer() :
	m_driverController(DriverConstants::kDriverControllerIndex)
{
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
	m_drivebase.SetDefaultCommand(frc2::RunCommand([this]
		{
			float driveSpeed = -m_driverController.GetLeftY();
			float turnSpeed = -m_driverController.GetRightX();

			m_drivebase.arcadeDrive(driveSpeed, turnSpeed, true);
		},
		{ &m_drivebase }
	).ToPtr());

	m_driverController.A().WhileTrue(frc2::RunCommand([this] {
		m_drivebase.arcadeDrive(0.2, 0, false);
	}, { &m_drivebase }).ToPtr());

	m_driverController.B().WhileTrue(frc2::RunCommand([this] {
		m_drivebase.arcadeDrive(0.4, 0, false);
	}, { &m_drivebase }).ToPtr());

	m_driverController.X().OnTrue(m_drivebase.moveCmd(5_m, 0_deg));

	m_driverController.Y().OnTrue(m_drivebase.moveCmd(-5_m, 0_deg));

	m_driverController.LeftBumper().WhileTrue(m_lift.moveCmd(-0.1f));
	m_driverController.RightBumper().WhileTrue(m_lift.moveCmd(0.1f));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	return this->m_lift.homeCmd().WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}
