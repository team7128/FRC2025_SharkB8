#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/EncoderSim.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>

#include "Constants.h"
#include "util/SparkPIDTuner.h"
#include "subsystems/Intake.h"

class Lift : public frc2::SubsystemBase
{
public:
	Lift(Intake &intake);

	virtual void Periodic() override;
	virtual void SimulationPeriodic() override;

	// Drive lift using percentage power
	void driveDirect(float speed);
	// Drive lift using a voltage, with an optional feedforward
	void driveVoltage(units::volt_t voltage, bool useFeedforward = true);

	// Get current height
	float getHeight();

	// Set current position as the bottom
	frc2::CommandPtr resetEncodersCmd();

	// Currently just resets the encoders
	frc2::CommandPtr homeCmd();
	// Moves at a set speed
	frc2::CommandPtr moveCmd(float speed);
	// Moves to a set position
	frc2::CommandPtr moveToPosCmd(float position, bool useFeedforward = true, bool end = true);
	// Stops movement, does not hold position
	frc2::CommandPtr stopCmd();
	// Hold the current position
	frc2::CommandPtr holdPosCmd();

	// Tune the feedforward parameters, set them in "Constants.h"
	// Set the tuning height with the "Lift Tune Height" on the dashboard
	frc2::CommandPtr tuneFeedforwardCmd();

	// Enable movement soft limits
	frc2::CommandPtr enableLimitsCmd();
	// Disable movement soft limits
	frc2::CommandPtr disableLimitsCmd();

private:
	rev::spark::SparkMax m_leftWinch, m_rightWinch;
	SparkPIDTuner m_sparkTuner;
	frc2::CommandPtr m_homeCmd;
	frc2::CommandPtr m_intakeMoveCmd;

	Intake &m_intake;

	void enableLimits();
	void disableLimits();

	float getCurrentFeedforward();

	struct
	{
		units::volt_t kG_guess = 0_V;
		frc::PIDController tuningController{ LiftConstants::tune_kP, LiftConstants::tune_kI, 0.0 };
		float positionTarget = 10.f;
		const std::string positionTargetKey = "Lift Tune Height";
	} m_feedforwardTuneData;

	// ----- Simulation Data -----

	frc::DCMotor m_simMotor = frc::DCMotor::NEO(2);
	frc::sim::ElevatorSim m_elevatorSim;
	rev::spark::SparkMaxSim m_simSpark;
};