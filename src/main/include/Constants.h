#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <frc/controller/ElevatorFeedforward.h>

#include <rev/ClosedLoopSlot.h>

namespace CANConstants
{
	constexpr inline unsigned int kSparkDriveIDs[4] = {
		1,  // Front left
		2,  // Back left
		3,  // Front right
		4   // Back right
	};

	inline constexpr unsigned int kLiftKrakenID = 1;
}

namespace DriverConstants
{
	constexpr inline unsigned int kDriverControllerIndex = 0;

	constexpr inline float kDriveMult = 1.0f,
		kTurnMult = 1.0f;
}

namespace DrivebaseConstants
{
	constexpr inline units::meter_t kDrivebaseWidth = 0.56_m;

	constexpr inline units::meter_t kWheelDiameter = 6_in;

	constexpr inline float kGearboxRatio = 1 / 8.45f;
}

namespace LiftConstants
{
	// Elevator feedforward constants
	// CURRENTLY JUST ESTIMATES
	inline constexpr float kG = 1.91f,
		kV = 0.39f,
		kA = 0.2f;
}