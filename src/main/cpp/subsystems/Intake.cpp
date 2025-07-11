#include "subsystems/Intake.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"

using namespace ctre::phoenix::motorcontrol;

Intake::Intake(bool enableAutoCommands) :
    m_beambreak(DIO_Constants::kBeambreakPort),
    m_intakeMotor(CANConstants::kIntakeVictorID),
    m_beambreakTrigger([this] { return m_beambreak.Get(); })
{
    SetDefaultCommand(this->Run([this] { m_intakeMotor.Set(ControlMode::Disabled, 0); }));

    m_beambreakTrigger.OnChange(frc2::cmd::RunOnce([this] { frc::SmartDashboard::PutBoolean("Beam Break", m_beambreak.Get()); }).IgnoringDisable(true));

    if (enableAutoCommands)
    {
        m_beambreakTrigger.OnFalse(partialIntakeCmd());
    }
}

bool Intake::isGamePieceClear() const
{
    return m_beambreak.Get();
}

frc2::CommandPtr Intake::drive(double speed)
{
    return this->Run([this, speed] { m_intakeMotor.Set(ControlMode::PercentOutput, speed); });
}

frc2::CommandPtr Intake::partialIntakeCmd()
{
    return this->RunOnce([this] { m_gamePieceState = GamePieceState::Partial; }).OnlyIf([this] { return m_gamePieceState == GamePieceState::None; })
        .AndThen(this->drive(0.8)).WithTimeout(0.4_s);
}

frc2::CommandPtr Intake::fullIntakeCmd()
{
    return this->drive(0.6).OnlyIf([this] { return !m_beambreak.Get(); })
            .Until([this] { return m_beambreak.Get(); })
        .AndThen(this->RunOnce([this] {
            m_intakeMotor.Set(ControlMode::Disabled, 0);
            m_gamePieceState = GamePieceState::Full;
        }));
}

frc2::CommandPtr Intake::releaseCmd()
{
    return this->drive(1.0).WithTimeout(0.5_s)
        .AndThen(this->RunOnce([this] { m_gamePieceState = GamePieceState::None; }));
}