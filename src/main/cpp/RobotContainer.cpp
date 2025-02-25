// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/RepeatCommand.h>

#include "Constants.h"

RobotContainer::RobotContainer() :
	m_driverController(DriverConstants::kDriverControllerIndex),
	m_liftDisableCmd(m_lift.stopCmd())
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

	m_driverController.POVUp().WhileTrue(m_lift.moveCmd(0.2f));
	m_driverController.POVDown().WhileTrue(m_lift.moveCmd(-0.2f));

	m_driverController.Back().OnTrue(m_lift.disableLimitsCmd());
	m_driverController.Back().OnFalse(m_lift.enableLimitsCmd().AndThen(m_lift.resetEncodersCmd()));

	m_driverController.LeftBumper().WhileTrue(m_climb.driveCmd(-0.4f).OnlyIf(std::bind(&Climb::isReleased, &m_climb)));
	m_driverController.RightBumper().WhileTrue(m_climb.driveCmd(0.4f).OnlyIf(std::bind(&Climb::isReleased, &m_climb)));

	m_driverController.Start().OnTrue(m_climb.releaseCmd());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	return this->m_lift.homeCmd().WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

void RobotContainer::SetupTestMode()
{
	static frc2::CommandPtr tuneCmd = m_lift.tuneFeedforwardCmd();
	frc::SmartDashboard::PutData("TuneLiftFeedforward", tuneCmd.get());
	frc::SmartDashboard::PutData("Lift Subsystem", &m_lift);

	frc::SmartDashboard::SetDefaultNumber(m_targetHeightKey, m_targetHeight);
	static frc2::CommandPtr heightTestCmd = frc2::cmd::Defer([this] {
			m_targetHeight = frc::SmartDashboard::GetNumber(m_targetHeightKey, m_targetHeight);
			return m_lift.moveToPosCmd(m_targetHeight);
		},
		{ &m_lift }
	);
	frc::SmartDashboard::PutData("Height Test Command", heightTestCmd.get());
}

void RobotContainer::Enable()
{
	m_liftDisableCmd.Cancel();
}

void RobotContainer::Disable()
{
	m_liftDisableCmd.Schedule();
}