#include "subsystems/CoralAligner.h"

#include "Constants.h"

using namespace ctre::phoenix::motorcontrol;

CoralAligner::CoralAligner() :
    m_motor(CANConstants::kAlignerTalonID)
{}

void CoralAligner::drive(float speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

frc2::CommandPtr CoralAligner::moveCmd(float speed)
{
    return this->Run(std::bind(&CoralAligner::drive, this, speed));
}

frc2::CommandPtr CoralAligner::returnCmd()
{
    return moveCmd(-1.f).Until([this] { return m_pdp.GetCurrent(AlignerConstants::kAlignerPDPSlot) > AlignerConstants::kAlignerStallCurrent; });
}