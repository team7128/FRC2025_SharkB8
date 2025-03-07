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

	inline constexpr unsigned int kClimbVictorIDs[2] = {
		1,
		2
	};

	inline constexpr unsigned int kAlignerTalonID = 1;
}

namespace UserConstants
{
	constexpr inline unsigned int kDriverControllerIndex = 0;
	
	constexpr inline unsigned int kLiftControllerIndex = 1;

	constexpr inline float kDriveMult = 0.4f,
		kTurnMult = 0.5f;
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
	inline float kLiftPresets[] = { 40.f, 69.f, 108.f };

	/**
	 * Feedforward values for the low and high points
	 * To deal with different weights on first and second stage
	 */
	inline constexpr float kGravityFeedforwardLow = 0.2f,
		kGravityFeedforwardHigh = 0.5f;
	// Height to change from low to high feedforward
	inline constexpr float kGravityHeightThreshold = 60.f;

	// Elevator PID parameters
	inline constexpr float kP = 0.01f,
		kI = 0.f,
		kD = 0.f;

	// In RPM and RPM/sec
	inline constexpr float kMaxSpeed = 3000.f,
		kMaxAccel = 20'000.f;
	
	// PID parameters for feedforward auto-tune
	inline constexpr float tune_kP = 0.08f,
		tune_kI = 0.03f;

	inline constexpr rev::spark::ClosedLoopSlot kPositionSlot = rev::spark::ClosedLoopSlot::kSlot0;
}

namespace ClimbConstants
{
	inline constexpr int kLimitSwitchPort = 0,
		kServoPort = 0;
}

namespace AlignerConstants
{
	inline constexpr int kAlignerPDPSlot = 9;
	inline constexpr float kAlignerStallCurrent = 15.f;
}