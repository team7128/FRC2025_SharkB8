#include "subsystems/Lift.h"

#include <numbers>

#include <frc2/command/WaitUntilCommand.h>

#include "Constants.h"

using namespace rev::spark;

Lift::Lift() :
	m_leftWinch(CANConstants::kLiftSparkIDs[0], SparkLowLevel::MotorType::kBrushless),
	m_rightWinch(CANConstants::kLiftSparkIDs[1], SparkLowLevel::MotorType::kBrushless)
{
	SparkBaseConfig sparkConfig;

	sparkConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
	sparkConfig.encoder.CountsPerRevolution(42);

	sparkConfig.closedLoop.P(LiftConstants::kP, LiftConstants::kPositionSlot);

	m_leftWinch.Configure(sparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	sparkConfig.encoder.Inverted(true);
	m_rightWinch.Configure(sparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	SetDefaultCommand(stopCmd());
}

void Lift::driveDirect(float speed)
{
	m_leftWinch.Set(speed);
	m_rightWinch.Set(speed);
}

frc2::CommandPtr Lift::homeCmd()
{
	return this->Run([this] { m_leftWinch.Set(-0.1); })
		.Until([] { return true; })
		.AndThen(this->RunOnce([this] {
				m_leftWinch.GetEncoder().SetPosition(0.0);
				m_rightWinch.GetEncoder().SetPosition(0.0);
			}));
}

frc2::CommandPtr Lift::moveCmd(float speed)
{
	return this->Run(std::bind(&Lift::driveDirect, this, speed));
}

frc2::CommandPtr Lift::moveToPosCmd(float position)
{
	return this->Run([this, position] {
		m_leftWinch.GetClosedLoopController().SetReference(position, SparkBase::ControlType::kPosition, LiftConstants::kPositionSlot, LiftConstants::kGravityFeedforward);
		m_rightWinch.GetClosedLoopController().SetReference(position, SparkBase::ControlType::kPosition, LiftConstants::kPositionSlot, LiftConstants::kGravityFeedforward);
	});
}

frc2::CommandPtr Lift::stopCmd()
{
	return this->Run([this] {
			m_leftWinch.StopMotor();
			m_rightWinch.StopMotor();
		});
}