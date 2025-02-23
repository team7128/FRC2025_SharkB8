#include "util/SparkPIDTuner.h"

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>

using namespace rev::spark;

SparkPIDTuner::SparkPIDTuner(SparkMax *spark, float P, float I, float D) :
	m_P(P),
	m_I(I),
	m_D(D),
	m_applyCmd(frc2::cmd::RunOnce([this] {
		SparkMaxConfig config;
		ClosedLoopSlot closedLoopSlot = ClosedLoopSlot(m_slot);

		config.closedLoop.P(m_P, closedLoopSlot)
			.I(m_I, closedLoopSlot)
			.D(m_D, closedLoopSlot);

		for (SparkMax *spark : m_sparks)
			spark->Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
	}))
{
	m_sparks.push_back(spark);
}

SparkPIDTuner::SparkPIDTuner(std::vector<SparkMax*> sparks, float P, float I, float D) :
	m_sparks(sparks),
	m_P(P),
	m_I(I),
	m_D(D),
	m_applyCmd(frc2::cmd::RunOnce([this] {
		if (m_slot < 0 || m_slot > 3)
			return;

		SparkMaxConfig config;
		ClosedLoopSlot closedLoopSlot = ClosedLoopSlot(m_slot);

		config.closedLoop.Pid(m_P, m_I, m_D, closedLoopSlot);

		for (SparkMax *spark : m_sparks)
		{
			spark->Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
		}
	}))
{}

void SparkPIDTuner::InitSendable(wpi::SendableBuilder &builder)
{
	builder.SetSmartDashboardType("SparkPIDTuner");
	builder.AddFloatProperty("P", [this] { return m_P; }, [this] (float newP) { m_P = newP; frc2::CommandScheduler::GetInstance().Schedule(m_applyCmd); });
	builder.AddFloatProperty("I", [this] { return m_I; }, [this] (float newI) { m_I = newI; frc2::CommandScheduler::GetInstance().Schedule(m_applyCmd); });
	builder.AddFloatProperty("D", [this] { return m_D; }, [this] (float newD) { m_D = newD; frc2::CommandScheduler::GetInstance().Schedule(m_applyCmd); });
	builder.AddIntegerProperty("Slot 0-3", [this] { return m_slot; }, [this] (float newSlot) { m_slot = newSlot; });
}