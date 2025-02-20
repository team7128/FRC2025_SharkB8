#include "subsystems/Odometry.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace rev::spark;

Odometry::Odometry(SparkRelativeEncoder &leftEncoder, SparkRelativeEncoder &rightEncoder) :
	m_leftEncoder(leftEncoder),
	m_rightEncoder(rightEncoder),
	m_odometry({}, 0_m, 0_m)
{}

void Odometry::reset()
{
	m_leftEncoder.SetPosition(0);
	m_rightEncoder.SetPosition(0);

	m_odometry.ResetPose(frc::Pose2d());
}

void Odometry::Periodic()
{
	m_odometry.Update({}, getLeftDistance(), getRightDistance());

	frc::SmartDashboard::PutNumber("Left Encoder", m_leftEncoder.GetPosition());
	frc::SmartDashboard::PutNumber("Right Encoder", m_rightEncoder.GetPosition());
	frc::SmartDashboard::PutNumber("Avg Dist", getAvgDistance().value());
	
	frc::SmartDashboard::PutNumber("Left Speed", m_leftEncoder.GetVelocity());
	frc::SmartDashboard::PutNumber("Right Speed", m_rightEncoder.GetVelocity());
	frc::SmartDashboard::PutNumber("Avg Speed", getAvgSpeed().value());
}

units::meter_t Odometry::getLeftDistance() const
{
	return units::meter_t(m_leftEncoder.GetPosition());
}

units::meter_t Odometry::getRightDistance() const
{
	return units::meter_t(m_rightEncoder.GetPosition());
}

units::meter_t Odometry::getAvgDistance() const
{
	return (getLeftDistance() + getRightDistance()) / 2;
}

units::meters_per_second_t Odometry::getLeftSpeed() const
{
	return units::meters_per_second_t(m_leftEncoder.GetVelocity());
}

units::meters_per_second_t Odometry::getRightSpeed() const
{
	return units::meters_per_second_t(m_rightEncoder.GetVelocity());
}

units::meters_per_second_t Odometry::getAvgSpeed() const
{
	return (getLeftSpeed() + getRightSpeed()) / 2;
}

units::degree_t Odometry::getAngle() const
{
	return m_odometry.GetPose().Rotation().Degrees();
}