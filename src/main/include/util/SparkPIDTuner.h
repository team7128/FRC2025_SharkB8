#pragma once

#include <wpi/sendable/Sendable.h>

#include <frc2/command/CommandPtr.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

class SparkPIDTuner : public wpi::Sendable
{
public:
	SparkPIDTuner(rev::spark::SparkMax *sparkMax, float P, float I, float D);
	SparkPIDTuner(std::vector<rev::spark::SparkMax*> sparks, float P, float I, float D);

	virtual void InitSendable(wpi::SendableBuilder &builder) override;

private:
	std::vector<rev::spark::SparkMax*> m_sparks;
	float m_P = 0.f, m_I = 0.f, m_D = 0.f;
	int m_slot = 0;
	frc2::CommandPtr m_applyCmd;
};