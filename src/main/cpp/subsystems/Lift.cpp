#include "subsystems/Lift.h"

#include <numbers>

#include <frc2/command/WaitUntilCommand.h>

#include "Constants.h"

using namespace ctre::phoenix6;

Lift::Lift() :
	m_kraken(1),
	m_homeRequest(-0.1),
	m_moveRequest(0),
	m_positionRequest(0_tr)
{
	configs::TalonFXConfiguration config;

	configs::Slot0Configs slot0Configs;

	slot0Configs.kG = LiftConstants::kG;
	slot0Configs.kV = LiftConstants::kV;
	slot0Configs.kA = LiftConstants::kA;

	slot0Configs.kP = 0.1;

	config.WithSlot0(slot0Configs);

	configs::SoftwareLimitSwitchConfigs limitConfig;

	limitConfig.WithForwardSoftLimitEnable(true)
		.WithForwardSoftLimitThreshold(1000_tr)
		.WithReverseSoftLimitEnable(true)
		.WithReverseSoftLimitThreshold(0_tr);

	config.WithSoftwareLimitSwitch(limitConfig);

	m_kraken.SetNeutralMode(signals::NeutralModeValue::Brake);

	m_kraken.GetConfigurator().Apply(config);

	m_homeRequest.WithLimitReverseMotion(false);

	m_moveRequest.WithLimitForwardMotion(true)
		.WithLimitReverseMotion(true);

	m_positionRequest.WithLimitForwardMotion(true)
		.WithLimitReverseMotion(true);

	SetDefaultCommand(stopCmd());
}

void Lift::driveDirect(float speed)
{
	m_moveRequest.WithOutput(speed);
	m_kraken.SetControl(m_moveRequest);
}

frc2::CommandPtr Lift::homeCmd()
{
	return this->Run([this] { m_kraken.SetControl(m_homeRequest); })
		.Until([] { return true; })
		.AndThen(this->RunOnce([this] { m_kraken.SetPosition(0_tr); }));
}

frc2::CommandPtr Lift::moveCmd(float speed)
{
	return this->Run(std::bind(&Lift::driveDirect, this, speed));
}

frc2::CommandPtr Lift::moveToPosCmd(units::meter_t height)
{
	return this->RunOnce([this, height] {
		m_positionRequest.WithPosition(height / (5_in * std::numbers::pi) * 1_tr);
	}).AndThen(this->Run([this] { m_kraken.SetControl(m_positionRequest); }).Until([this] { return m_kraken.GetClosedLoopError().GetValue() < 0.5; }));
}

frc2::CommandPtr Lift::stopCmd()
{
	return this->Run([this] { m_kraken.SetControl(m_stopRequest); });
}