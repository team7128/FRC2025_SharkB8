#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/EncoderSim.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>

#include "Constants.h"

class Lift : public frc2::SubsystemBase
{
public:
	Lift();

	virtual void SimulationPeriodic() override;

	void driveDirect(float speed);
	void driveVoltage(units::volt_t voltage);

	frc2::CommandPtr homeCmd();
	frc2::CommandPtr moveCmd(float speed);
	frc2::CommandPtr moveToPosCmd(float position);
	frc2::CommandPtr stopCmd();

	frc2::CommandPtr tuneFeedforwardCmd(units::volt_t initialGuess = units::volt_t(LiftConstants::kGravityFeedforward));

private:
	rev::spark::SparkMax m_leftWinch, m_rightWinch;

	void enableFollow();
	void disableFollow();
	frc2::CommandPtr enableFollowCmd();
	frc2::CommandPtr disableFollowCmd();

	struct
	{
		units::volt_t kG_guess = 0_V;
		frc::PIDController positionController{ LiftConstants::kP / 12, 0.0, LiftConstants::kD / 12 };
		frc::PIDController tuningController{ 0.005, 0.0, 0.0 };
		const float positionTarget = 10.f;
	} m_feedforwardTuneData;

	// ----- Simulation Data -----

	frc::DCMotor m_simMotor = frc::DCMotor::NEO(2);
	frc::sim::ElevatorSim m_elevatorSim;
	rev::spark::SparkMaxSim m_simSpark;
};