#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rev/SparkMax.h>

#include <units/length.h>
#include <units/angle.h>

#include "subsystems/Odometry.h"

enum class ControlType
{
	Direct,
	PID
};

class Drivebase : public frc2::SubsystemBase
{
public:
	Drivebase();

	virtual void Periodic() override;

	void arcadeDrive(float driveSpeed, float turnSpeed, bool square);

	void stop();

	frc2::CommandPtr driveCmd(float speed);
	frc2::CommandPtr driveTimedCmd(float speed, units::second_t time);
	frc2::CommandPtr moveCmd(units::meter_t distance, units::degree_t angle);
	frc2::CommandPtr driveDumbCmd(float speed, units::meter_t distance);

	frc2::CommandPtr stopCmd();

private:
	rev::spark::SparkMax m_motorFL, m_motorBL, m_motorFR, m_motorBR;

	frc::DifferentialDrive m_diffDrive;

	Odometry m_odometry;

	frc::ProfiledPIDController<units::meter> m_drivePID;
	frc::ProfiledPIDController<units::degree> m_turnPID;
};