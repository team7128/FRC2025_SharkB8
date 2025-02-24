#include "subsystems/Climb.h"

#include "Constants.h"

using namespace ctre::phoenix::motorcontrol;

Climb::Climb() :
    m_climbMotor1(CANConstants::kClimbVictorIDs[0]),
    m_climbMotor2(CANConstants::kClimbVictorIDs[1])
{
    m_climbMotor1.SetInverted(InvertType::InvertMotorOutput);
    m_climbMotor2.SetInverted(InvertType::FollowMaster);

    m_climbMotor2.Follow(m_climbMotor1);

    SetDefaultCommand(stopCmd());
}

void Climb::drive(float speed)
{
    m_climbMotor1.Set(ControlMode::PercentOutput, speed);
}

frc2::CommandPtr Climb::driveCmd(float speed)
{
    return this->Run(std::bind(&Climb::drive, this, speed));
}

void Climb::stop()
{
    m_climbMotor1.Set(ControlMode::Disabled, 0.0);
}

frc2::CommandPtr Climb::stopCmd()
{
    return this->Run(std::bind(&Climb::stop, this));
}