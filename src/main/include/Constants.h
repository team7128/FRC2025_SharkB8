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

	inline constexpr unsigned int kLiftSparkIDs[2] = {
		5,	// Left
		6	// Right
	};
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
	inline constexpr float kGravityFeedforward{ 1.91f };
	inline constexpr float kP = 0.1f;

	inline constexpr rev::spark::ClosedLoopSlot kPositionSlot = rev::spark::ClosedLoopSlot::kSlot0;
}