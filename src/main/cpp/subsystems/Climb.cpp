#include "subsystems/Climb.h"

#include <frc/smartdashboard/SmartDashboard.h>	

#include <frc2/command/Commands.h>

#include "Constants.h"

using namespace ctre::phoenix::motorcontrol;

Climb::Climb() :
	m_climbMotor1(CANConstants::kClimbVictorIDs[0]),
	m_climbMotor2(CANConstants::kClimbVictorIDs[1]),
	m_limitSwitch(ClimbConstants::kLimitSwitchPort),
	m_intakeRelease(ClimbConstants::kServoPort)
{
	m_climbMotor1.ConfigOpenloopRamp(2.0);
	m_climbMotor1.SetInverted(InvertType::InvertMotorOutput);

	m_climbMotor2.SetInverted(InvertType::FollowMaster);
	m_climbMotor2.Follow(m_climbMotor1);

	m_intakeRelease.SetAngle(180.0);

	frc::SmartDashboard::PutData("Ramp Servo", &m_intakeRelease);

	SetDefaultCommand(stopCmd());
}

void Climb::drive(float speed)
{
	m_climbMotor1.Set(ControlMode::PercentOutput, speed);
}

frc2::CommandPtr Climb::driveCmd(float speed)
{
	return this->Run(std::bind(&Climb::drive, this, speed)).Until(std::bind(&frc::DigitalInput::Get, &m_limitSwitch));
}

void Climb::stop()
{
	m_climbMotor1.Set(ControlMode::Disabled, 0.0);
}

frc2::CommandPtr Climb::stopCmd()
{
	return this->Run(std::bind(&Climb::stop, this));
}

frc2::CommandPtr Climb::releaseCmd()
{
	return frc2::cmd::RunOnce([this] {
		m_intakeRelease.SetAngle(110.0);
		m_released = true;
	});
}

frc2::CommandPtr Climb::unreleaseCmd()
{
	return frc2::cmd::RunOnce([this] {
		m_intakeRelease.SetAngle(180.0);
		m_released = false;
	});
}