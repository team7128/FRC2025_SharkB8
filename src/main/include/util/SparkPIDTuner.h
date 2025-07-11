#pragma once

#include <wpi/sendable/Sendable.h>

#include <frc2/command/CommandPtr.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

// A dashboard tuner for spark PID and MaxMotion parameters
class SparkPIDTuner : public wpi::Sendable
{
public:
	SparkPIDTuner(rev::spark::SparkMax *sparkMax);
	SparkPIDTuner(rev::spark::SparkMax *sparkMax, float P, float I, float D, float maxSpeed, float maxAccel);
	SparkPIDTuner(std::vector<rev::spark::SparkMax*> sparks, float P, float I, float D, float maxSpeed, float maxAccel);

	virtual void InitSendable(wpi::SendableBuilder &builder) override;

	void applySettings();

private:
	std::vector<rev::spark::SparkMax*> m_sparks;
	float m_P = 0.f, m_I = 0.f, m_D = 0.f;
	float m_maxSpeed = 0.f, m_maxAccel = 0.f;
	int m_slot = 0;
};