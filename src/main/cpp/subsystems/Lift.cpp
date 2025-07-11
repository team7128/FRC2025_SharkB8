#include "subsystems/Lift.h"

#include <numbers>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/Commands.h>

#include <rev/config/SparkMaxConfig.h>

#include <wpi/raw_os_ostream.h>

#include "Constants.h"

using namespace rev::spark;

Lift::Lift(Intake &intake) :
	m_leftWinch(CANConstants::kLiftSparkIDs[0], SparkLowLevel::MotorType::kBrushless),
	m_rightWinch(CANConstants::kLiftSparkIDs[1], SparkLowLevel::MotorType::kBrushless),
	m_sparkTuner({ &m_leftWinch, &m_rightWinch }, LiftConstants::kP, LiftConstants::kI, LiftConstants::kD, LiftConstants::kMaxSpeed, LiftConstants::kMaxAccel),
	m_homeCmd(homeCmd()),
	m_intakeMoveCmd(intake.fullIntakeCmd()),
	m_intake(intake)
	, m_elevatorSim(m_simMotor, 12.75, 20_kg, 25_mm, 0_m, 2_m, true, 0_m),
	m_simSpark(&m_leftWinch, &m_simMotor)
{
	SparkMaxConfig sparkConfig;

	sparkConfig
		.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
		.Inverted(false);

	sparkConfig.softLimit
		.ReverseSoftLimit(0.0)
		.ReverseSoftLimitEnabled(true);

	sparkConfig.closedLoop
		.Pid(LiftConstants::kP, LiftConstants::kI, LiftConstants::kD, LiftConstants::kPositionSlot)
		.IZone(3, LiftConstants::kPositionSlot);
	
	sparkConfig.closedLoop.maxMotion
		.MaxVelocity(LiftConstants::kMaxSpeed, LiftConstants::kPositionSlot)
		.MaxAcceleration(LiftConstants::kMaxAccel, LiftConstants::kPositionSlot)
		.PositionMode(MAXMotionConfig::MAXMotionPositionMode::kMAXMotionTrapezoidal, LiftConstants::kPositionSlot)
		.AllowedClosedLoopError(0.1, LiftConstants::kPositionSlot);

	m_leftWinch.Configure(sparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	sparkConfig.Inverted(true);

	m_rightWinch.Configure(sparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	m_feedforwardTuneData.tuningController.SetTolerance(0.02, 0.002);

	SetDefaultCommand(
		holdPosCmd()
			// When getting driven, automatically extend the game piece
			.FinallyDo([this] {
				m_intakeMoveCmd.Schedule();
			})
	);

	frc::SmartDashboard::PutData("Lift Spark PID", &m_sparkTuner);
	frc::SmartDashboard::PutData("Home Lift", m_homeCmd.get());
	frc::SmartDashboard::PutNumber(m_feedforwardTuneData.positionTargetKey, m_feedforwardTuneData.positionTarget);
}

void Lift::Periodic()
{
	frc::SmartDashboard::PutNumber("Lift Height L", m_leftWinch.GetEncoder().GetPosition());
	frc::SmartDashboard::PutNumber("Lift Height R", m_rightWinch.GetEncoder().GetPosition());
	frc::SmartDashboard::PutNumber("Lift Speed L", m_leftWinch.GetEncoder().GetVelocity());
	frc::SmartDashboard::PutNumber("Lift Speed R", m_rightWinch.GetEncoder().GetVelocity());
	frc::SmartDashboard::PutBoolean("Lift Limit L", m_leftWinch.GetReverseLimitSwitch().Get());
	frc::SmartDashboard::PutBoolean("Lift Limit R", m_rightWinch.GetReverseLimitSwitch().Get());
	frc::SmartDashboard::PutNumber("Lift Out L", m_leftWinch.GetAppliedOutput());
	frc::SmartDashboard::PutNumber("Lift Out R", m_rightWinch.GetAppliedOutput());
	frc::SmartDashboard::PutNumber("Lift Current L", m_leftWinch.GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Lift Current R", m_rightWinch.GetOutputCurrent());
}

void Lift::SimulationPeriodic()
{
	m_elevatorSim.SetInput(frc::Vectord<1>{ m_simSpark.GetAppliedOutput() * frc::RobotController::GetInputVoltage() });

	m_elevatorSim.Update(20_ms);

	m_simSpark.iterate((m_elevatorSim.GetVelocity() / 25_mm * 1_tr).convert<units::rpm>().value(), frc::RobotController::GetInputVoltage(), 0.02);
}

void Lift::driveDirect(float speed)
{
	if (m_intake.isGamePieceClear())
	{
		m_leftWinch.Set(speed);
		m_rightWinch.Set(speed);
	}
	else
	{
		m_leftWinch.Disable();
		m_rightWinch.Disable();
	}
}

void Lift::driveVoltage(units::volt_t voltage, bool useFeedforward)
{
	if (useFeedforward)
		voltage += units::volt_t(getCurrentFeedforward());

	if (m_intake.isGamePieceClear())
	{
		m_leftWinch.SetVoltage(voltage);
		m_rightWinch.SetVoltage(voltage);
	}
	else
	{
		m_leftWinch.Disable();
		m_rightWinch.Disable();
	}
}

float Lift::getHeight()
{
	return (m_leftWinch.GetEncoder().GetPosition() + m_rightWinch.GetEncoder().GetPosition()) / 2.f;
}

frc2::CommandPtr Lift::resetEncodersCmd()
{
	return this->RunOnce([this] {
		m_leftWinch.GetEncoder().SetPosition(-0.1);
		m_rightWinch.GetEncoder().SetPosition(-0.1);
	});
}

frc2::CommandPtr Lift::homeCmd()
{
	return disableLimitsCmd()
		.AndThen(moveCmd(-0.1f)
			.Until([] { return true; }))	// TODO: Add auto homing condition
		.AndThen(resetEncodersCmd())
		.AndThen(enableLimitsCmd())
		.AndThen(moveToPosCmd(0.0));
}

frc2::CommandPtr Lift::moveCmd(float speed)
{
	return frc2::cmd::WaitUntil([this] { return m_intake.isGamePieceClear(); }).AndThen(this->Run(std::bind(&Lift::driveDirect, this, speed)));
}

frc2::CommandPtr Lift::moveToPosCmd(float position, bool useFeedforward, bool end)
{
	return frc2::cmd::WaitUntil([this] { return m_intake.isGamePieceClear(); })
		.AndThen(this->RunOnce([this] {
			m_leftWinch.GetClosedLoopController().SetIAccum(0.0);
			m_rightWinch.GetClosedLoopController().SetIAccum(0.0);
		}))
		.AndThen(this->Run([this, position, useFeedforward] {
			float feedforward = useFeedforward ? getCurrentFeedforward() : 0.f;
			feedforward = 0;
			m_leftWinch.GetClosedLoopController().SetReference(position, SparkBase::ControlType::kMAXMotionPositionControl, LiftConstants::kPositionSlot, feedforward);
			m_rightWinch.GetClosedLoopController().SetReference(position, SparkBase::ControlType::kMAXMotionPositionControl, LiftConstants::kPositionSlot, feedforward);
		}))
			.Until([this, position, end] {
				double avgVel = (m_leftWinch.GetEncoder().GetVelocity() + m_rightWinch.GetEncoder().GetVelocity()) / 2;
				return (abs(getHeight() - position) < 0.5) && (abs(avgVel) < 20) && end;
			});
}

frc2::CommandPtr Lift::stopCmd()
{
	return this->Run([this] {
		m_leftWinch.StopMotor();
		m_rightWinch.StopMotor();
	});
}

frc2::CommandPtr Lift::holdPosCmd()
{
	return this->Defer([this] { return moveToPosCmd(getHeight(), true, false); });
}

frc2::CommandPtr Lift::tuneFeedforwardCmd()
{
	frc::SmartDashboard::PutData("Feedforward Tune PID", &m_feedforwardTuneData.tuningController);

	std::vector<frc2::CommandPtr> tuningSequence;

	tuningSequence.push_back(this->RunOnce([this] {
		m_feedforwardTuneData.kG_guess = 0_V;
		m_feedforwardTuneData.tuningController.Reset();
		m_feedforwardTuneData.positionTarget = frc::SmartDashboard::GetNumber(m_feedforwardTuneData.positionTargetKey, 10.f);
	}));
	
	tuningSequence.push_back(this->Run([this] {
		double pidOut = m_feedforwardTuneData.tuningController.Calculate(m_leftWinch.GetEncoder().GetPosition(), m_feedforwardTuneData.positionTarget);

		m_feedforwardTuneData.kG_guess = units::volt_t(pidOut);
		driveVoltage(units::volt_t(pidOut), false);

		frc::SmartDashboard::PutNumber("LiftFeedforward", m_feedforwardTuneData.kG_guess.value());
		frc::SmartDashboard::PutNumber("Position Out", pidOut);
	}).Until([this] { return m_feedforwardTuneData.tuningController.AtSetpoint(); }));

	tuningSequence.push_back(this->RunOnce([this] { wpi::outs() << "Final lift feedforward value: " << std::to_string(m_feedforwardTuneData.kG_guess.value()) << "\n"; }));

	return frc2::cmd::Sequence(std::move(tuningSequence));
}

float Lift::getCurrentFeedforward()
{
	float height = m_leftWinch.GetEncoder().GetPosition();

	return height < LiftConstants::kGravityHeightThreshold ? LiftConstants::kGravityFeedforwardLow : LiftConstants::kGravityFeedforwardHigh;
}

void Lift::enableLimits()
{
	static bool configMade = false;
	static SparkMaxConfig enableLimitConfig;
	if (!configMade)
	{
		enableLimitConfig.softLimit.ReverseSoftLimitEnabled(true);
		configMade = true;
	}

	m_leftWinch.Configure(enableLimitConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
	m_rightWinch.Configure(enableLimitConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
}

void Lift::disableLimits()
{
	static bool configMade = false;
	static SparkMaxConfig disableLimitConfig;
	if (!configMade)
	{
		disableLimitConfig.softLimit.ReverseSoftLimitEnabled(false);
		configMade = true;
	}

	m_leftWinch.Configure(disableLimitConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
	m_rightWinch.Configure(disableLimitConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
}

frc2::CommandPtr Lift::enableLimitsCmd()
{
	return frc2::cmd::RunOnce(std::bind(&Lift::enableLimits, this));
}

frc2::CommandPtr Lift::disableLimitsCmd()
{
	return frc2::cmd::RunOnce(std::bind(&Lift::disableLimits, this));
}