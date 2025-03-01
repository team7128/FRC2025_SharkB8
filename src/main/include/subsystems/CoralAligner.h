#pragma once

#include <frc/PowerDistribution.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

class CoralAligner : public frc2::SubsystemBase
{
public:
    CoralAligner();

    void drive(float speed);

    frc2::CommandPtr moveCmd(float speed);
    frc2::CommandPtr returnCmd();

private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_motor;
    frc::PowerDistribution m_pdp;
};