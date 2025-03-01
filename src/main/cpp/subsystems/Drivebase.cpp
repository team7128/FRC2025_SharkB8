#include "subsystems/Drivebase.h"

#include <numbers>

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <wpi/raw_os_ostream.h>

#include "Constants.h"

using namespace rev::spark;

Drivebase::Drivebase() :
	m_motorFL(CANConstants::kSparkDriveIDs[0], SparkLowLevel::MotorType::kBrushless),
	m_motorBL(CANConstants::kSparkDriveIDs[1], SparkLowLevel::MotorType::kBrushless),
	m_motorFR(CANConstants::kSparkDriveIDs[2], SparkLowLevel::MotorType::kBrushless),
	m_motorBR(CANConstants::kSparkDriveIDs[3], SparkLowLevel::MotorType::kBrushless),
	m_diffDrive(m_motorFL, m_motorFR),
	m_odometry(m_motorFL.GetEncoder(), m_motorFR.GetEncoder()),
	m_drivePID(
		DrivebaseConstants::kDriveP, DrivebaseConstants::kDriveI, DrivebaseConstants::kDriveD,
		{ DrivebaseConstants::kAutoMaxSpeed, DrivebaseConstants::kAutoMaxAccel }
	),
	m_turnPID(
		DrivebaseConstants::kTurnP, DrivebaseConstants::kTurnI, DrivebaseConstants::kTurnD,
		{ DrivebaseConstants::kAutoMaxTurnSpeed, DrivebaseConstants::kAutoMaxTurnAccel }
	)
{
	SparkBaseConfig leftSparkConfig, rightSparkConfig;

	m_odometry.reset();

	leftSparkConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
	// leftSparkConfig.SmartCurrentLimit(20, 10, 0);

	units::meter_t distancePerRev = DrivebaseConstants::kWheelDiameter * std::numbers::pi * DrivebaseConstants::kGearboxRatio;

	leftSparkConfig.encoder.PositionConversionFactor(distancePerRev.value())
		.VelocityConversionFactor(distancePerRev.value() / 60.0);

	rightSparkConfig.Apply(leftSparkConfig);

	leftSparkConfig.Inverted(false);
	rightSparkConfig.Inverted(true);

	m_motorFL.Configure(leftSparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
	m_motorFR.Configure(rightSparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	leftSparkConfig.Follow(m_motorFL, false);
	rightSparkConfig.Follow(m_motorFR, false);

	m_motorBL.Configure(leftSparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
	m_motorBR.Configure(rightSparkConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

	m_drivePID.SetTolerance(0.1_m, 1_mps);
	m_drivePID.SetIZone(1);

	m_turnPID.SetTolerance(15_deg, 45_deg_per_s);
	m_turnPID.EnableContinuousInput(-180_deg, 180_deg);
	m_turnPID.SetIZone(45);

	SetDefaultCommand(stopCmd());
}

void Drivebase::Periodic()
{
	frc::SmartDashboard::PutNumber("Drivebase Avg Dist", m_odometry.getAvgDistance().value());
}

void Drivebase::arcadeDrive(float driveSpeed, float turnSpeed, bool square)
{
	m_diffDrive.ArcadeDrive(driveSpeed, turnSpeed, square);
}

void Drivebase::stop()
{
	m_diffDrive.StopMotor();
}

frc2::CommandPtr Drivebase::driveCmd(float speed)
{
	return this->Run(std::bind(&Drivebase::arcadeDrive, this, speed, 0.0, false));
}

frc2::CommandPtr Drivebase::driveTimedCmd(float speed, units::second_t time)
{
	return driveCmd(speed).WithTimeout(time);
}

frc2::CommandPtr Drivebase::moveCmd(units::meter_t distance, units::degree_t angle)
{
	return this->RunOnce([this, distance, angle] {
		m_drivePID.Reset(m_odometry.getAvgDistance());
		m_turnPID.Reset(m_odometry.getAngle());
		
		m_drivePID.SetGoal(distance + m_odometry.getAvgDistance());
		m_turnPID.SetGoal(angle);
	}).AndThen(this->Run([this] {
		auto driveSpeed = m_drivePID.Calculate(m_odometry.getAvgDistance());
		auto turnSpeed = m_turnPID.Calculate(m_odometry.getAngle());

		arcadeDrive(driveSpeed, turnSpeed, false);
	}).Until([this] { return m_drivePID.AtGoal() && m_turnPID.AtGoal(); }));
}

frc2::CommandPtr Drivebase::driveDumbCmd(float speed, units::meter_t distance)
{
	return this->RunOnce([this] {
		m_odometry.reset();
	}).AndThen(this->Run([this, speed] {
		arcadeDrive(speed, 0.f, false);
	}).Until([this, distance] { return std::abs(m_odometry.getAvgDistance().value()) > std::abs(distance.value()); }));
}

frc2::CommandPtr Drivebase::stopCmd()
{
	return this->Run(std::bind(&Drivebase::stop, this));
}