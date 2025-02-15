// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/GenericHID.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/NetworkButton.h>

#include "Constants.h"
#include "DashBoardValue.h"

#include "ISubsystemAccess.h"

#include "subsystems/ElevatorSubsystem.h"

using namespace frc;
using namespace frc2;

enum ESideSelected 
  {
    Unselected,
    LeftSide,
    RightSide
  };

class RobotContainer : public ISubsystemAccess
{
 public:
  RobotContainer();

  void Periodic();

  // ISubsystemAcces Implementation
  DriveSubsystem&            GetDrive() override { return m_drive; }
  VisionSubsystem&           GetVision() override { return m_vision; }
  ElevatorSubsystem&         GetElevator() override { return m_elevator; }
  CoralManipulatorSubsystem& GetCoral() override { return m_coral; }
  IntakeSubsystem&           GetIntake() override { return m_intake; }
#ifdef LED
  LEDSubsystem&              GetLED() override { return m_led; }
#endif
  // END ISubsystemAcces Implementation
  
  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }
  Command*                   GetAutonomousCommand();
  // enum EAutoPath
  // {
  //     kAutoPathDefault
  //   , kAutoPathTestAuto = kAutoPathDefault
  //   , kAutoPathDoNothing
  //   // Keep the emun in sync with the LUT
  // };
  // std::vector<std::string> m_pathPlannerLUT
  // { 
  //     "Test Auto" 
  //   , "DoNothingAuto"       // These strings are the names of the PathPlanner .path files
  // };
//  frc::SendableChooser<EAutoPath> m_chooser;
  frc::SendableChooser<Command*> m_chooser;
  void StartUp() { m_intake.AlignIntake(); }

  // ConfigureRobotLEDs called by Robot class, passes enabled state via dashboard value "Robot Enabled"
  void ConfigureRobotLEDs();

  // Stop all motors (currently elevator only)
  void StopAll();

  std::shared_ptr<PathPlannerPath> GetOnTheFlyPath();
  std::shared_ptr<PathPlannerPath> GetOnTheFlyPathWithPrint();

 private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
  void ConfigSecondaryButtonBindings();
#define USE_BUTTON_BOX
#ifdef USE_BUTTON_BOX
  void ConfigButtonBoxBindings();
#endif

  void SetSideSelected(ESideSelected sideSelected);

  // Used for OnTheFlyPaths
  static frc2::CommandPtr GetFollowPathCommand() { return m_pThis->GetFollowPathCommandImpl();};
  frc2::CommandPtr GetFollowPathCommandImpl();
  static RobotContainer* m_pThis; 

  // The robot's subsystems and commands are defined here...
  
  DriveSubsystem m_drive;
  VisionSubsystem m_vision;
  ElevatorSubsystem m_elevator;
  CoralManipulatorSubsystem m_coral;
  IntakeSubsystem m_intake;
#ifdef LED
  LEDSubsystem m_led;
#endif

  CommandXboxController m_primaryController{0};
  CommandXboxController m_secondaryController{1};
#ifdef USE_BUTTON_BOX
  CommandXboxController m_buttonController{3};
#endif

  SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s, -3 / 1_s};
  SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s, -3 / 1_s};
  SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
  // TODO Make sure field relative starts how the drive team wants
  bool m_fieldRelative = true;
  bool m_isAutoRunning = false;

  ESideSelected m_sideSelected = Unselected;

  frc2::InstantCommand m_toggleFieldRelative{[this] { 
    m_fieldRelative = !m_fieldRelative; 
    frc::SmartDashboard::PutBoolean("FieldRelative", m_fieldRelative);
    }, {}};
  
  frc2::InstantCommand m_toggleSlowSpeed{[this] { 
    m_drive.ToggleSlowSpeed();
    }, {}};

  frc2::InstantCommand m_setHighSpeedCmd{[this] {
    m_drive.SetSlowSpeed(false);
  }, {&m_drive}};

  // Tag 3 coordinates
  // frc2::InstantCommand m_resetOdo{[this] {m_drive.ResetOdometry({11.56_m, 8.12_m, 90_deg});}, {&m_drive}};
  // frc2::InstantCommand m_resetOdo{[this] {m_drive.ResetOdometry({530.49_in + 8.75_in, 130.17_in - 15.16_in, 120_deg});}, {&m_drive}};
  frc2::InstantCommand m_resetOdo{[this] {m_drive.ResetOdometry({481.39_in - 17.5_in, 158.5_in, 0_deg});}, {&m_drive}};

  frc2::InstantCommand m_elevL4{[this] 
  { 
    m_drive.SetSlowSpeed(true);
    m_elevator.GoToPosition(c_defaultL4Turns); }, {&m_elevator} 
  };
  frc2::InstantCommand m_elevL3{[this] { m_drive.SetSlowSpeed(true); m_elevator.GoToPosition(c_defaultL3Turns); }, {&m_elevator} };
  frc2::InstantCommand m_elevL2{[this] { m_drive.SetSlowSpeed(true); m_elevator.GoToPosition(c_defaultL2Turns); }, {&m_elevator} };
  frc2::InstantCommand m_elevL2_3{[this] { m_drive.SetSlowSpeed(true); m_elevator.GoToPosition(c_algaeRemovalL2_3); }, {&m_elevator} };
  frc2::InstantCommand m_elevL3_4{[this] { m_drive.SetSlowSpeed(true); m_elevator.GoToPosition(c_algaeRemovalL3_4); }, {&m_elevator} };
  frc2::InstantCommand m_setL1{[this] { m_elevator.SetPresetLevel(L1); }, {&m_elevator} };
  frc2::InstantCommand m_setL2{[this] { m_elevator.SetPresetLevel(L2); }, {&m_elevator} };
  frc2::InstantCommand m_setL3{[this] { m_elevator.SetPresetLevel(L3); }, {&m_elevator} };
  frc2::InstantCommand m_setL4{[this] { m_elevator.SetPresetLevel(L4); }, {&m_elevator} };

  frc2::InstantCommand m_setRight{[this] { SetSideSelected(RightSide); }, {} };
  frc2::InstantCommand m_setLeft{[this] { SetSideSelected(LeftSide); }, {} };

  frc2::InstantCommand m_elevReset{[this] { m_elevator.ElevatorReset(); }, {&m_elevator} };
  frc2::InstantCommand m_elevRelPosUp{[this] { m_elevator.GotoPositionRel(1.0); }, {&m_elevator} };
  frc2::InstantCommand m_elevRelPosDown{[this] { m_elevator.GotoPositionRel(-1.0); }, {&m_elevator} };

  frc2::InstantCommand m_intakeAlign{[this] { m_intake.AlignIntake(); }, {&m_intake} };
  frc2::InstantCommand m_intakePark{[this] { m_intake.ParkIntakeForClimb(); }, {&m_intake} };
  frc2::InstantCommand m_intakeParkAtZero{[this] { m_intake.ParkIntakeAtZero(); }, {&m_intake} };

  frc2::InstantCommand m_coralEject{[this]
  { 
    bool down = (m_elevator.GetCurrentPosition() < 2.0);
    m_coral.EjectCoral(down); 
  }, {&m_coral} };
  frc2::InstantCommand m_coralStop{[this] { m_coral.Stop(); }, {&m_coral} };
  frc2::InstantCommand m_coralRetract{[this] { m_coral.RetractCoral(L1); }, {&m_coral} };
  frc2::InstantCommand m_coralDeployManip{[this] {m_coral.DeployManipulator(); }, {&m_coral} };
  frc2::InstantCommand m_coralRetractManip{[this] {m_coral.RetractManipulator(); }, {&m_coral} };

  frc2::InstantCommand m_rumblePrimary{[this] { m_primaryController.SetRumble(GenericHID::RumbleType::kBothRumble, 1); }, {} };
  frc2::InstantCommand m_stopRumblePrimary{[this] { m_primaryController.SetRumble(GenericHID::RumbleType::kBothRumble, 0); }, {} };

  NetworkButton m_netButtonTest{"NetButtons", "Test"};
  // DashBoardValue<bool> m_dbvFieldRelative{"Drive", "FieldRelative", false};

  PathConstraints m_pathConstraints { 1.0_mps, 1.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq };
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
};
