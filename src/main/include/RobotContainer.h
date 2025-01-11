// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"

#include "ISubsystemAccess.h"

using namespace frc;
using namespace frc2;

class RobotContainer : public ISubsystemAccess
{
 public:
  RobotContainer();

  void Periodic();
  // ISubsystemAcces Implementation
  DriveSubsystem&        GetDrive() override { return m_drive; }
  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }
  frc2::CommandPtr GetAutonomousCommand();

 private:
private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
  void ConfigSecondaryButtonBindings();

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_drive;
// #define USE_XBOX
#ifdef USE_XBOX
  CommandXboxController m_primaryController{0};
  CommandXboxController m_secondaryController{1};
#else
  frc2::CommandJoystick m_primaryController{0};
  frc2::CommandJoystick m_secondaryController{1};
#endif
  SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s, -3 / 2_s};
  SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s, -3 / 3_s};
  SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
  SlewRateLimiter<units::scalar> m_yawRotationLimiter{3 / 1_s};
  // TODO Make sure field relative starts how the drive team wants
  bool m_fieldRelative = true;
  bool m_isAutoRunning = false;

  frc2::InstantCommand m_toggleFieldRelative{[this] { m_fieldRelative = !m_fieldRelative; }, {}};

//#define TEST_WHEEL_CONTROL
#ifdef TEST_WHEEL_CONTROL
#define DISABLE_DRIVING
  frc2::InstantCommand m_wheelsForward{[this] { GetDrive().WheelsForward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsLeft{[this] { GetDrive().WheelsLeft(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsBackward{[this] { GetDrive().WheelsBackward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsRight{[this] { GetDrive().WheelsRight(); }, {&m_drive} };
#endif  // TEST_WHEEL_CONTROL
};
