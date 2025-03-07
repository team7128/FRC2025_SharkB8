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
#include <cameraserver/CameraServer.h>

#include "Constants.h"

RobotContainer::RobotContainer() :
	m_driverController(UserConstants::kDriverControllerIndex),
	m_liftController(UserConstants::kLiftControllerIndex),
	m_liftDisableCmd(m_lift.stopCmd())
{
	ConfigureBindings();
	SetupTestCommands();
	frc::CameraServer::StartAutomaticCapture(0);
	frc::CameraServer::StartAutomaticCapture(1);
	
}

void RobotContainer::ConfigureBindings()
{
	// ===== DRIVER CONTROLLER =====

	//float liftedSpeed = (0.0f);

	m_drivebase.SetDefaultCommand(frc2::RunCommand([this]
		{
			
			float driveSpeed = (-m_driverController.GetLeftY()) * std::sqrt(UserConstants::kDriveMult);
			float turnSpeed = (-m_driverController.GetRightX()) * std::sqrt(UserConstants::kTurnMult);

			m_drivebase.arcadeDrive(driveSpeed, turnSpeed, true);

			

	/*if (m_lift.m_feedforwardTuneData.positionTarget > 5)
			{
				liftedSpeed = 0.6f;
			}
			else 
			{
			liftedSpeed = 1.0f;
			} */
	
		},
		{ &m_drivebase }
	).ToPtr());

	



	/*
	m_driverController.A().WhileTrue(frc2::RunCommand([this] {
		m_drivebase.arcadeDrive(0.2f, 0, false);
	}, { &m_drivebase }).ToPtr());

	m_driverController.B().WhileTrue(frc2::RunCommand([this] {
		m_drivebase.arcadeDrive(0.4f, 0, false);
	}, { &m_drivebase }).ToPtr());
	*/
	//m_driverController.X().OnTrue(m_drivebase.moveCmd(5_m, 0_deg));

	//m_driverController.Y().OnTrue(m_drivebase.moveCmd(-5_m, 0_deg));

	// Climb controls
	// Right bumper to bring arm out
	// A to bring cage in slowly
	// Left bumper to bring cage in at full speed
	m_driverController.RightBumper().WhileTrue(m_climb.driveCmd(0.2f));
	m_driverController.A().WhileTrue(m_climb.driveCmd(-0.4f));
	m_driverController.LeftBumper().WhileTrue(m_climb.driveCmd(-1.0f));

	// Release the intake ramp by pressing Start and Back
	frc2::Trigger rampReleaseTrigger = m_liftController.Start() && m_liftController.Back();
	rampReleaseTrigger.OnTrue(m_climb.releaseCmd());
	rampReleaseTrigger.OnFalse(m_climb.unreleaseCmd());

	// ===== LIFT CONTROLLER =====/

	// Drive at variable speeds with left stick up/down
	m_liftController.AxisMagnitudeGreaterThan(1, 0.1)
		.WhileTrue(m_lift.Run([this] {
			m_lift.driveDirect(-m_liftController.GetLeftY());
		}));	

	// Drive at set speed with D-pad up/down
	//m_liftController.POVUp().WhileTrue(m_lift.moveCmd(0.3f));
	//m_liftController.POVDown().WhileTrue(m_lift.moveCmd(-0.3f));

	// Hold start to disable lower limit and realign the bottom of the lift
	m_liftController.Back().OnTrue(m_lift.disableLimitsCmd());
	m_liftController.Back().OnFalse(m_lift.enableLimitsCmd().AndThen(m_lift.resetEncodersCmd()));

	// A, X and Y for preset heights
	// B to reset to the bottom
	m_liftController.A().OnTrue(m_lift.moveToPosCmd(LiftConstants::kLiftPresets[0]));
	m_liftController.X().OnTrue(m_lift.moveToPosCmd(LiftConstants::kLiftPresets[1]));
	m_liftController.Y().OnTrue(m_lift.moveToPosCmd(LiftConstants::kLiftPresets[2]));
	m_liftController.B().OnTrue(m_lift.moveToPosCmd(0.f));

	// Move coral aligner left/right with the right stick left/right
	m_liftController.AxisMagnitudeGreaterThan(4, 0.1)
		.WhileTrue(m_aligner.Run([this] {
			m_aligner.drive(m_liftController.GetRightX());
		}));

	// Return aligner to intake position with start
	m_liftController.Start().OnTrue(m_aligner.returnCmd());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	return m_drivebase.driveTimedCmd(-0.20, 5.5_s);
}

void RobotContainer::SetupTestCommands()
{
	static frc2::CommandPtr tuneCmd = m_lift.tuneFeedforwardCmd();
	frc::SmartDashboard::PutData("TuneLiftFeedforward", tuneCmd.get());
	frc::SmartDashboard::PutData("Lift Subsystem", &m_lift);

	frc::SmartDashboard::SetDefaultNumber(m_targetHeightKey, m_targetHeight);
	static frc2::CommandPtr heightTestCmd = m_lift.Defer([this] {
			m_targetHeight = frc::SmartDashboard::GetNumber(m_targetHeightKey, m_targetHeight);
			return m_lift.moveToPosCmd(m_targetHeight);
	});
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