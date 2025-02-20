#include <frc/controller/ElevatorFeedforward.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

class Lift : public frc2::SubsystemBase
{
public:
    Lift();

    void driveDirect(float speed);

    frc2::CommandPtr homeCmd();
    frc2::CommandPtr moveCmd(float speed);
    frc2::CommandPtr moveToPosCmd(units::meter_t height);
    frc2::CommandPtr stopCmd();

private:
    ctre::phoenix6::hardware::TalonFX m_kraken;

    ctre::phoenix6::controls::DutyCycleOut m_homeRequest, m_moveRequest;
    ctre::phoenix6::controls::PositionVoltage m_positionRequest;
    ctre::phoenix6::controls::StaticBrake m_stopRequest;
};