#include "AutoSequences.h"

#include <frc2/command/Commands.h>

#include "Constants.h"

frc2::CommandPtr Autos::autoScore(int height, Drivebase &drivebase, Lift &lift, Intake &intake)
{
    std::vector<frc2::CommandPtr> sequence;
    // Drive back a small amount to clear reef
    sequence.push_back(drivebase.driveTimedCmd(-0.15, 0.2_s));
    // Go up to the correct height
    sequence.push_back(lift.moveToPosCmd(LiftConstants::kLiftPresets[height]));
    // Make sure the intake finishes moving
    sequence.push_back(intake.releaseCmd());

    // If going for the top, add a small "bump" to nudge the coral onto the reef
    if (height == 2)
    {
        sequence.push_back(lift.moveToPosCmd(LiftConstants::kLiftPresets[3]));
    }

    // Reset the lift position
    sequence.push_back(lift.moveToPosCmd(0.0));

    return frc2::cmd::Sequence(std::move(sequence));
}