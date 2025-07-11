#include "AutoSequences.h"

#include <frc2/command/Commands.h>

#include "Constants.h"

frc2::CommandPtr Autos::autoScore(int height, Drivebase &drivebase, Lift &lift, Intake &intake)
{
    std::vector<frc2::CommandPtr> sequence;
    sequence.push_back(drivebase.driveTimedCmd(-0.1, 0.4_s));
    sequence.push_back(lift.moveToPosCmd(LiftConstants::kLiftPresets[height]));
    sequence.push_back(frc2::cmd::WaitUntil([&lift] { return lift.GetCurrentCommand() == lift.GetDefaultCommand(); }));
    sequence.push_back(intake.releaseCmd());

    if (height == 2)
    {
        sequence.push_back(lift.moveToPosCmd(LiftConstants::kLiftPresets[3]));
    }

    sequence.push_back(lift.moveToPosCmd(0.0));

    return frc2::cmd::Sequence(std::move(sequence));
}