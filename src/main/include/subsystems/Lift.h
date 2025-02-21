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

class Lift : public frc2::SubsystemBase
{
public:
	Lift();

	virtual void SimulationPeriodic() override;

	void driveDirect(float speed);
	void driveVoltage(units::volt_t voltage, bool useFeedforward = true);

	frc2::CommandPtr homeCmd();
	frc2::CommandPtr moveCmd(float speed);
	frc2::CommandPtr moveToPosCmd(float position, bool useFeedforward = true);
	frc2::CommandPtr stopCmd();
	frc2::CommandPtr holdPosCmd();

	frc2::CommandPtr tuneFeedforwardCmd();

private:
	rev::spark::SparkMax m_leftWinch, m_rightWinch;
	SparkPIDTuner m_sparkTuner;

	void enableFollow();
	void disableFollow();
	frc2::CommandPtr enableFollowCmd();
	frc2::CommandPtr disableFollowCmd();

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