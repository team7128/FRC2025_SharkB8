#pragma once

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
	/**
	 * Feedforward values for the low and high points
	 * To deal with different weights on first and second stage
	 */
	inline constexpr float kGravityFeedforwardLow = 0.445f,
		kGravityFeedforwardHigh = 0.f;
	// Height to change from low to high feedforward
	inline constexpr float kGravityHeightThreshold = 40.f;

	// Elevator PID parameters
	inline constexpr float kP = 0.5f,
		kI = 0.f,
		kD = 0.01f;
	
	// PID parameters for feedforward auto-tune
	inline constexpr float tune_kP = 0.8f,
		tune_kI = 0.1f;

	inline constexpr rev::spark::ClosedLoopSlot kPositionSlot = rev::spark::ClosedLoopSlot::kSlot0;
}