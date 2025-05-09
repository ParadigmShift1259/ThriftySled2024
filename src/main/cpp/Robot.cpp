// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include <cameraserver/CameraServer.h>

void Robot::RobotInit()
{
  frc::CameraServer::StartAutomaticCapture();  
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit()
{
  m_container.StopAll();
  DataLogManager::Stop();
  m_enabled = false;
  frc::SmartDashboard::PutBoolean("Robot Enabled", m_enabled);
  m_container.ConfigureRobotLEDs();
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit()
{
  m_enabled = true;
  frc::SmartDashboard::PutBoolean("Robot Enabled", m_enabled);
  m_container.ConfigureRobotLEDs();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
  DataLogManager::LogNetworkTables(true);
  DataLogManager::LogConsoleOutput(true);
  DataLogManager::Start();
  // Record both DS control and joystick data
  DriverStation::StartDataLog(DataLogManager::GetLog());

  m_container.SetHighSpeed();
  m_container.StartUp();

  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  DataLogManager::LogNetworkTables(true);
  DataLogManager::LogConsoleOutput(true);
  DataLogManager::Start();
  // Record both DS control and joystick data
  DriverStation::StartDataLog(DataLogManager::GetLog());

  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
