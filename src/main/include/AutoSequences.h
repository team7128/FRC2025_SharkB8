#include <frc2/Command/CommandPtr.h>

#include "subsystems/Drivebase.h"
#include "subsystems/Lift.h"
#include "subsystems/Intake.h"

namespace Autos
{
    frc2::CommandPtr autoScore(int height, Drivebase &drivebase, Lift &lift, Intake &intake);
}