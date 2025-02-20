#pragma once

#include <frc/kinematics/DifferentialDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include <rev/SparkRelativeEncoder.h>

class Odometry : public frc2::SubsystemBase
{
public:
	Odometry(rev::spark::SparkRelativeEncoder &leftEncoder, rev::spark::SparkRelativeEncoder &rightEncoder);

	void Periodic() override;

	void reset();

	units::meter_t getLeftDistance() const;
	units::meter_t getRightDistance() const;
	units::meter_t getAvgDistance() const;

	units::meters_per_second_t getLeftSpeed() const;
	units::meters_per_second_t getRightSpeed() const;
	units::meters_per_second_t getAvgSpeed() const;

	units::degree_t getAngle() const;

private:
	rev::spark::SparkRelativeEncoder &m_leftEncoder, &m_rightEncoder;

	frc::DifferentialDriveOdometry m_odometry;
};