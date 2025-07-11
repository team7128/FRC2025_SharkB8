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
#include "AutoSequences.h"

RobotContainer::RobotContainer() :
	m_driverController(UserConstants::kDriverControllerIndex)
	, m_liftController(UserConstants::kLiftControllerIndex)
	, m_intake(true)
	, m_lift(m_intake)
	, m_liftDisableCmd(m_lift.stopCmd())
{
	ConfigureBindings();
	SetupTestCommands();
	
	frc::CameraServer::StartAutomaticCapture(0);
	frc::CameraServer::StartAutomaticCapture(1);

	m_autoChooser.SetDefaultOption("Mobility", AutoSequence::Mobility);
	m_autoChooser.AddOption("Trough Bump", AutoSequence::TroughBump);
	m_autoChooser.AddOption("Score Coral", AutoSequence::CoralScore);

	frc::SmartDashboard::PutData("Auto Sequence", &m_autoChooser);
}

void RobotContainer::ConfigureBindings()
{
	// ===== DRIVER CONTROLLER =====

	m_drivebase.SetDefaultCommand(frc2::RunCommand([this]
		{
			float elevatorSlowdown = m_lift.getHeight() > 5.f ? UserConstants::kLiftExtendSlowdown : 1.f;

			float driveSpeed = (-m_driverController.GetLeftY()) * std::sqrt(UserConstants::kDriveMult * elevatorSlowdown);
			float turnSpeed = (-m_driverController.GetRightX()) * std::sqrt(UserConstants::kTurnMult * elevatorSlowdown);

			m_drivebase.arcadeDrive(driveSpeed, turnSpeed, true);
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
	frc2::Trigger rampReleaseTrigger = m_driverController.Start() && m_driverController.Back();
	rampReleaseTrigger.OnTrue(m_climb.releaseCmd());
	rampReleaseTrigger.OnFalse(m_climb.unreleaseCmd());

	// ===== LIFT CONTROLLER =====/

	// Use left stick up/down to drive lift manually
	m_liftController.AxisMagnitudeGreaterThan(1, 0.1)
		.WhileTrue(m_lift.Run([this] {
			m_lift.driveDirect(-m_liftController.GetLeftY() * UserConstants::kLiftManualSensitivity);
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
	m_liftController.B().OnTrue(m_lift.moveToPosCmd(0));

	m_liftController.LeftBumper().WhileTrue(m_intake.drive(1.0));
	m_liftController.RightBumper().WhileTrue(m_intake.drive(-1.0));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	switch (m_autoChooser.GetSelected())
	{
	case AutoSequence::Mobility:
		return m_drivebase.driveTimedCmd(-0.4, 1_s);
	case AutoSequence::TroughBump:
		return m_drivebase.driveTimedCmd(-0.2, 5.5_s);
	case AutoSequence::CoralScore:
		return m_drivebase.driveTimedCmd(0.2, 5.5_s).AndThen(Autos::autoScore(2, m_drivebase, m_lift, m_intake));
	}

	return frc2::cmd::None();
}

void RobotContainer::SetupTestCommands()
{
	static frc2::CommandPtr tuneCmd = m_lift.tuneFeedforwardCmd();
	frc::SmartDashboard::PutData("TuneLiftFeedforward", tuneCmd.get());
	frc::SmartDashboard::PutData("Lift Subsystem", &m_lift);

	static frc2::CommandPtr partialIntakeCmd = m_intake.partialIntakeCmd(),
		fullIntakeCmd = m_intake.fullIntakeCmd(),
		releaseCmd = m_intake.releaseCmd();
	frc::SmartDashboard::PutData("Partial Intake", partialIntakeCmd.get());
	frc::SmartDashboard::PutData("Full Intake", fullIntakeCmd.get());
	frc::SmartDashboard::PutData("Release Game Piece", releaseCmd.get());

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