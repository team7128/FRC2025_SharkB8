// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandScheduler.h>

Robot::Robot()
{
	EnableLiveWindowInTest(true);
}

void Robot::RobotPeriodic()
{
	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit()
{
	m_container.Disable();
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit()
{
	m_container.Enable();
}

void Robot::AutonomousInit()
{
	m_autonomousCommand = m_container.GetAutonomousCommand();

	if (m_autonomousCommand)
	{
		m_autonomousCommand->Schedule();
	}
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
	if (m_autonomousCommand)
	{
		m_autonomousCommand->Cancel();
	}
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
