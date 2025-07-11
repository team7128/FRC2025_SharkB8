#pragma once

#include <frc/DigitalInput.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

class Intake : public frc2::SubsystemBase
{
public:
    Intake(bool enableAutoCommands);

    // Move intake at a set speed
    frc2::CommandPtr drive(double speed);

    bool isGamePieceClear() const;

    // Partially grab the game piece
    frc2::CommandPtr partialIntakeCmd();
    // Extend the game piece to clear the lift
    frc2::CommandPtr fullIntakeCmd();
    // Throw out the game piece
    frc2::CommandPtr releaseCmd();

private:
    enum class GamePieceState
    {
        None, Partial, Full
    };

    frc::DigitalInput m_beambreak;
    ctre::phoenix::motorcontrol::can::VictorSPX m_intakeMotor;

    GamePieceState m_gamePieceState = GamePieceState::None;
    frc2::Trigger m_beambreakTrigger;
};