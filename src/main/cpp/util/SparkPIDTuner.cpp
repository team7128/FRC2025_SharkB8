#include "util/SparkPIDTuner.h"

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>

using namespace rev::spark;

SparkPIDTuner::SparkPIDTuner(SparkMax *spark) :
	SparkPIDTuner(spark
		, spark->configAccessor.closedLoop.GetP()
		, spark->configAccessor.closedLoop.GetI()
		, spark->configAccessor.closedLoop.GetD()
		, spark->configAccessor.closedLoop.maxMotion.GetMaxVelocity()
		, spark->configAccessor.closedLoop.maxMotion.GetMaxAcceleration()
	)
{}

SparkPIDTuner::SparkPIDTuner(SparkMax *spark, float P, float I, float D, float maxSpeed, float maxAccel) :
	SparkPIDTuner(std::vector<SparkMax*>({ spark }), P, I, D, maxSpeed, maxAccel)
{}

SparkPIDTuner::SparkPIDTuner(std::vector<SparkMax*> sparks, float P, float I, float D, float maxSpeed, float maxAccel) :
	m_sparks(sparks),
	m_P(P),
	m_I(I),
	m_D(D),
	m_maxSpeed(maxSpeed),
	m_maxAccel(maxAccel)
{}

void SparkPIDTuner::InitSendable(wpi::SendableBuilder &builder)
{
	builder.SetSmartDashboardType("SparkPIDTuner");
	builder.AddFloatProperty("P", [this] { return m_P; }, [this] (float newP) { m_P = newP; applySettings(); });
	builder.AddFloatProperty("I", [this] { return m_I; }, [this] (float newI) { m_I = newI; applySettings(); });
	builder.AddFloatProperty("D", [this] { return m_D; }, [this] (float newD) { m_D = newD; applySettings(); });
	builder.AddFloatProperty("Max Speed", [this] { return m_maxSpeed; }, [this] (float newSpeed) { m_maxSpeed = newSpeed; applySettings(); });
	builder.AddFloatProperty("Max Accel", [this] { return m_maxAccel; }, [this] (float newAccel) { m_maxAccel = newAccel; applySettings(); });
	builder.AddIntegerProperty("Slot 0-3", [this] { return m_slot; }, [this] (float newSlot) { m_slot = newSlot; });
}

void SparkPIDTuner::applySettings()
{
	if (m_slot < 0 || m_slot > 3)
			return;

	SparkMaxConfig config;
	ClosedLoopSlot closedLoopSlot = ClosedLoopSlot(m_slot);

	config.closedLoop.Pid(m_P, m_I, m_D, closedLoopSlot);
	config.closedLoop.maxMotion.MaxVelocity(m_maxSpeed, closedLoopSlot)
		.MaxAcceleration(m_maxAccel, closedLoopSlot);

	for (SparkMax *spark : m_sparks)
	{
		spark->Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kNoPersistParameters);
	}
}