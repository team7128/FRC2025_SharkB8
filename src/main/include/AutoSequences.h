#include <frc2/Command/CommandPtr.h>

#include "subsystems/Drivebase.h"
#include "subsystems/Lift.h"
#include "subsystems/Intake.h"

namespace Autos
{
    // Scores on a set level, assuming the robot starts aligned up agains the reef
    frc2::CommandPtr autoScore(int height, Drivebase &drivebase, Lift &lift, Intake &intake);
}