// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"

#include "ISubsystemAccess.h"

#include "subsystems/ElevatorSubsystem.h"

using namespace frc;
using namespace frc2;

class RobotContainer : public ISubsystemAccess
{
 public:
  RobotContainer();

  void Periodic();
  // ISubsystemAcces Implementation
  DriveSubsystem&        GetDrive() override { return m_drive; }
  VisionSubsystem&       GetVision() override { return m_vision; }
  ElevatorSubsystem&       GetElevator() override {return m_elevator; }
  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }
  frc2::CommandPtr GetAutonomousCommand();

  void StopAll();

  std::shared_ptr<PathPlannerPath> GetOnTheFlyPath();

 private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
  void ConfigSecondaryButtonBindings();
  void SetUpLogging();

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_drive;
  VisionSubsystem m_vision;
  ElevatorSubsystem m_elevator;

#define USE_XBOX
#ifdef USE_XBOX
  CommandXboxController m_primaryController{0};
  CommandXboxController m_secondaryController{1};
#else
  frc2::CommandJoystick m_primaryController{0};
  frc2::CommandJoystick m_secondaryController{1};
#endif
  SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s, -3 / 1_s};
  SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s, -3 / 1_s};
  SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
  SlewRateLimiter<units::scalar> m_yawRotationLimiter{3 / 1_s};
  // TODO Make sure field relative starts how the drive team wants
  bool m_fieldRelative = false;
  bool m_isAutoRunning = false;

  frc2::InstantCommand m_toggleFieldRelative{[this] { 
    m_fieldRelative = !m_fieldRelative; 
    frc::SmartDashboard::PutBoolean("FieldRelative", m_fieldRelative);
    }, {}};
  
  // Tag 3 coordinates
  // frc2::InstantCommand m_resetOdo{[this] {m_drive.ResetOdometry({11.56_m, 8.12_m, 90_deg});}, {&m_drive}};
  // frc2::InstantCommand m_resetOdo{[this] {m_drive.ResetOdometry({530.49_in + 8.75_in, 130.17_in - 15.16_in, 120_deg});}, {&m_drive}};
  frc2::InstantCommand m_resetOdo{[this] {m_drive.ResetOdometry({481.39_in - 17.5_in, 158.5_in, 0_deg});}, {&m_drive}};
  frc2::InstantCommand m_printPath{[this] {GetOnTheFlyPath();}, {}};

  frc2::InstantCommand m_elevL4{[this] 
  { 
    double elevHeight = 38.0;//frc::SmartDashboard::GetNumber("elevHeight", 0.0);
    m_elevator.GoToPosition(elevHeight); }, {&m_elevator} 
  };
  frc2::InstantCommand m_elevL3{[this] { m_elevator.GoToPosition(15.5); }, {&m_elevator} };
  frc2::InstantCommand m_elevL2{[this] { m_elevator.GoToPosition(2.0); }, {&m_elevator} };

  frc2::InstantCommand m_elevReset{[this] { m_elevator.ElevatorReset(); }, {&m_elevator} };
  frc2::InstantCommand m_elevRelPosUp{[this] { m_elevator.GotoPositionRel(1.0); }, {&m_elevator} };
  frc2::InstantCommand m_elevRelPosDown{[this] { m_elevator.GotoPositionRel(-1.0); }, {&m_elevator} };

  //std::optional<frc2::CommandPtr> m_pathCmd;
  PathConstraints m_pathConstraints { 0.25_mps, 0.25_mps_sq, 90_deg_per_s, 180_deg_per_s_sq };
  std::shared_ptr<PathPlannerPath> m_path;
  double m_targetX;
  double m_targetY;
  double m_targetRot;

//#define TEST_WHEEL_CONTROL
#ifdef TEST_WHEEL_CONTROL
#define DISABLE_DRIVING
  frc2::InstantCommand m_wheelsForward{[this] { GetDrive().WheelsForward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsLeft{[this] { GetDrive().WheelsLeft(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsBackward{[this] { GetDrive().WheelsBackward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsRight{[this] { GetDrive().WheelsRight(); }, {&m_drive} };
#endif  // TEST_WHEEL_CONTROL

  using TranslationLog = wpi::log::StructLogEntry<frc::Translation2d>;
  TranslationLog m_logPath;
  TranslationLog m_logPoses;
};
