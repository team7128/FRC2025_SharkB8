#include "subsystems/Lift.h"

#include <numbers>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/Commands.h>

#include <rev/config/SparkMaxConfig.h>

#include <wpi/raw_os_ostream.h>

#include "Constants.h"

using namespace rev::spark;

Lift::Lift() :
	m_leftWinch(CANConstants::kLiftSparkIDs[0], SparkLowLevel::MotorType::kBrushless),
	m_rightWinch(CANConstants::kLiftSparkIDs[1], SparkLowLevel::MotorType::kBrushless),
	m_elevatorSim(m_simMotor, 12.75, 10_kg, 25_mm, 0_m, 2_m, true, 0_m),
	m_simSpark(&m_leftWinch, &m_simMotor)
{
	SparkMaxConfig sparkConfig;

	sparkConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
		.Inverted(true);

	sparkConfig.closedLoop.P(LiftConstants::kP, LiftConstants::kPositionSlot)
		.D(LiftConstants::kD, LiftConstants::kPositionSlot);

	m_leftWinch.Configure(sparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	sparkConfig.encoder.PositionConversionFactor(-1.0)
		.VelocityConversionFactor(-1.0);
	
	m_rightWinch.Configure(sparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	m_feedforwardTuneData.tuningController.SetTolerance(0.02, 0.001);
	m_feedforwardTuneData.positionController.SetTolerance(0.2, 0.1);

	SetDefaultCommand(stopCmd());
}

void Lift::SimulationPeriodic()
{
	m_elevatorSim.SetInput(frc::Vectord<1>{ m_simSpark.GetAppliedOutput() * frc::RobotController::GetInputVoltage() });

	m_elevatorSim.Update(20_ms);

	frc::SmartDashboard::PutNumber("SimLiftHeight", m_elevatorSim.GetPosition().value());

	m_simSpark.iterate((m_elevatorSim.GetVelocity() / 25_mm * 1_tr).convert<units::rpm>().value(), frc::RobotController::GetInputVoltage(), 0.02);
}

void Lift::driveDirect(float speed)
{
	m_leftWinch.Set(speed);
	m_rightWinch.Set(speed);
}

void Lift::driveVoltage(units::volt_t voltage)
{
	m_leftWinch.SetVoltage(voltage);
	m_rightWinch.SetVoltage(voltage);
}

frc2::CommandPtr Lift::homeCmd()
{
	return disableFollowCmd()
		.AndThen(this->Run([this] { m_leftWinch.Set(-0.1); })
			.Until([] { return true; }))
		.AndThen(this->RunOnce([this] {
			m_leftWinch.GetEncoder().SetPosition(0.0);
			m_rightWinch.GetEncoder().SetPosition(0.0);
		}));
}

frc2::CommandPtr Lift::moveCmd(float speed)
{
	return enableFollowCmd().AndThen(this->Run(std::bind(&Lift::driveDirect, this, speed)));
}

frc2::CommandPtr Lift::moveToPosCmd(float position)
{
	return disableFollowCmd()
		.AndThen(this->Run([this, position] {
			m_leftWinch.GetClosedLoopController().SetReference(position, SparkBase::ControlType::kPosition, LiftConstants::kPositionSlot, LiftConstants::kGravityFeedforward);
			m_rightWinch.GetClosedLoopController().SetReference(position, SparkBase::ControlType::kPosition, LiftConstants::kPositionSlot, LiftConstants::kGravityFeedforward);
		}));
}

frc2::CommandPtr Lift::stopCmd()
{
	return this->Run([this] {
			m_leftWinch.StopMotor();
			m_rightWinch.StopMotor();
		});
}

frc2::CommandPtr Lift::tuneFeedforwardCmd(units::volt_t initialGuess)
{
	std::vector<frc2::CommandPtr> tuningSequence;

	tuningSequence.push_back(enableFollowCmd());

	tuningSequence.push_back(this->RunOnce([this, initialGuess] {
		m_feedforwardTuneData.kG_guess = initialGuess;
		m_feedforwardTuneData.positionController.Reset();
		m_feedforwardTuneData.tuningController.Reset();
	}));
	
	tuningSequence.push_back(this->Run([this] {
		double positionOutput = m_feedforwardTuneData.positionController.Calculate(m_leftWinch.GetEncoder().GetPosition(), m_feedforwardTuneData.positionTarget);
		double tuningOutput = m_feedforwardTuneData.tuningController.Calculate(positionOutput, 0.0);

		m_feedforwardTuneData.kG_guess -= units::volt_t(tuningOutput);

		driveVoltage(units::volt_t(positionOutput) + m_feedforwardTuneData.kG_guess);

		frc::SmartDashboard::PutNumber("LiftFeedforward", m_feedforwardTuneData.kG_guess.value());
	}).Until([this] { return m_feedforwardTuneData.tuningController.AtSetpoint() && m_feedforwardTuneData.positionController.AtSetpoint(); }));

	tuningSequence.push_back(this->RunOnce([this] { wpi::outs() << "Final lift feedforward value: " << std::to_string(m_feedforwardTuneData.kG_guess.value()) << "\n"; }));

	return frc2::cmd::Sequence(std::move(tuningSequence));
}

void Lift::enableFollow()
{
	m_rightWinch.ResumeFollowerMode();
}

void Lift::disableFollow()
{
	m_rightWinch.PauseFollowerMode();
}

frc2::CommandPtr Lift::enableFollowCmd()
{
	return this->RunOnce(std::bind(&Lift::enableFollow, this));
}

frc2::CommandPtr Lift::disableFollowCmd()
{
	return this->RunOnce(std::bind(&Lift::disableFollow, this));
}