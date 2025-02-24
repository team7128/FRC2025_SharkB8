#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

class Climb : public frc2::SubsystemBase
{
public:
    Climb();

    void drive(float speed);
    frc2::CommandPtr driveCmd(float speed);

    void stop();
    frc2::CommandPtr stopCmd();

private:
    ctre::phoenix::motorcontrol::can::VictorSPX m_climbMotor1, m_climbMotor2;
};