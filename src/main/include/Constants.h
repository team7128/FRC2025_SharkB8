#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <frc/controller/ElevatorFeedforward.h>

#include <rev/ClosedLoopSlot.h>

namespace DIO_Constants
{
	constexpr inline unsigned int kBeambreakPort = 0;
}

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

	inline constexpr unsigned int kClimbVictorIDs[2] = {
		1,
		2
	};

	inline constexpr unsigned int kAlignerTalonID = 1;

	inline constexpr unsigned int kIntakeVictorID = 3;
}

namespace UserConstants
{
	constexpr inline unsigned int kDriverControllerIndex = 0;
	
	constexpr inline unsigned int kLiftControllerIndex = 1;

	constexpr inline float kDriveMult = 0.4f,
		kTurnMult = 0.5f;

	// Drive speed multiplier for when the lift is extended
	constexpr inline float kLiftExtendSlowdown = 0.4f;

	constexpr inline double kLiftManualSensitivity = 0.3;
}

namespace DrivebaseConstants
{
	constexpr inline units::meter_t kDrivebaseWidth = 0.56_m;

	constexpr inline units::meter_t kWheelDiameter = 6_in;

	constexpr inline float kGearboxRatio = 1 / 8.45f;

	constexpr inline units::meters_per_second_t kAutoMaxSpeed = 1_mps;
	constexpr inline units::meters_per_second_squared_t kAutoMaxAccel = 2_mps_sq;

	constexpr inline float kDriveP = 0.2f,
		kDriveI = 0.f,
		kDriveD = 0.02f;

	constexpr inline units::degrees_per_second_t kAutoMaxTurnSpeed = 120_deg_per_s;
	constexpr inline units::degrees_per_second_squared_t kAutoMaxTurnAccel = 500_deg_per_s_sq;

	constexpr inline float kTurnP = 0.02f,
		kTurnI = 0.f,
		kTurnD = 0.004f;
}

namespace LiftConstants
{
	inline float kLiftPresets[] = { 19.f, 49.f, 97.f, 103.f };

	/**
	 * Feedforward values for the low and high points
	 * To deal with different weights on first and second stage
	 */
	inline constexpr float kGravityFeedforwardLow = 0.27f,
		kGravityFeedforwardHigh = 0.34f;
	// Height to change from low to high feedforward
	inline constexpr float kGravityHeightThreshold = 60.f;

	// Elevator PID parameters
	inline constexpr float kP = 0.2f,
		kI = 0.f,
		kD = 0.6f;

	// In RPM and RPM/sec
	inline constexpr float kMaxSpeed = 5000.f,
		kMaxAccel = kMaxSpeed * 4;
	
	// PID parameters for feedforward auto-tune
	inline constexpr float tune_kP = 0.08f,
		tune_kI = 0.03f;

	inline constexpr rev::spark::ClosedLoopSlot kPositionSlot = rev::spark::ClosedLoopSlot::kSlot0;
}

namespace ClimbConstants
{
	inline constexpr int kLimitSwitchPort = 1,
		kServoPort = 0;
}

namespace AlignerConstants
{
	inline constexpr int kAlignerPDPSlot = 9;
	inline constexpr float kAlignerStallCurrent = 15.f;
}