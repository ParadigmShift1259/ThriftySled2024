// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/GenericHID.h>
#include <frc/Timer.h>

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
  ClimberSubsystem&          GetClimber() override { return m_climber; }
#ifdef LED
  LEDSubsystem&              GetLED() override { return m_led; }
#endif
  // END ISubsystemAcces Implementation
  
  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }
  Command*                   GetAutonomousCommand();

  frc::SendableChooser<Command*> m_chooser;
  void StartUp()
  { 
    m_intake.AlignIntake();
    m_timer.Reset();
    m_timer.Start();
  }

  // ConfigureRobotLEDs called by Robot class, passes enabled state via dashboard value "Robot Enabled"
  void ConfigureRobotLEDs();

  // Stop all motors (currently elevator only)
  void StopAll();
  void SetHighSpeed(){m_drive.SetSlowSpeed(false);}

  std::shared_ptr<PathPlannerPath> GetOnTheFlyPath();

 private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
#define USE_SECOND_XBOX_CONTROLLER
#ifdef USE_SECOND_XBOX_CONTROLLER
  void ConfigSecondaryButtonBindings();
#endif
#define USE_BUTTON_BOX
// #define USE_POSITION_PID
#ifdef USE_BUTTON_BOX
  void ConfigButtonBoxBindings();
#endif
  void ConfigureNetworkButtons();

  bool GetTagPose(Pose2d& tagPose);
  void AreWeInTheSweetSpot();
  void SetSideSelected(ESideSelected sideSelected);
  void CalcTargetPoses();
  void PrintTargetPoses();

  // Used for OnTheFlyPaths, the FollowPathCommand wants a free function to build the path
  // This trick converts a static member fucntion into a call to an instance function
  static frc2::CommandPtr GetFollowPathCommand() { return m_pThis->GetFollowPathCommandImpl();};
  frc2::CommandPtr GetFollowPathCommandImpl();
  static RobotContainer* m_pThis; 

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_drive;
  VisionSubsystem m_vision;
  ElevatorSubsystem m_elevator;
  CoralManipulatorSubsystem m_coral;
  IntakeSubsystem m_intake;
  ClimberSubsystem m_climber;
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
  bool m_fieldRelative = false;//true;
  bool m_isAutoRunning = false;
  DashBoardValue<bool> m_dbvFieldRelative{"Drive", "FieldRelative", m_fieldRelative};
  DashBoardValue<double> m_dbvOnTheFlyPathAccel{"Drive", "OTFPaccel", 3.0};
  DashBoardValue<double> m_dbvDistToTag{"Vision", "Dist2tag", -1.0};
  DashBoardValue<bool> m_dbv1Meter{"Vision", "1Meter", false};  
  DashBoardValue<bool> m_dbv2Meter{"Vision", "2Meter", false};  
  DashBoardValue<bool> m_dbv3Meter{"Vision", "3Meter", false};  
  DashBoardValue<bool> m_dbv4Meter{"Vision", "4Meter", false};  

  DashBoardValue<bool> m_dbvRunIntakeStartup{"Intake", "RunStartup", false};
  DashBoardValue<bool> m_dbvRunCoralRetract{"Coral", "CoralRetract", false};
  DashBoardValue<bool> m_dbvRunElevL2{"Elevator", "GoL2", false};
  DashBoardValue<bool> m_dbvRunElevL3{"Elevator", "GoL3", false};
  DashBoardValue<bool> m_dbvRunElevL4{"Elevator", "GoL4", false};
  DashBoardValue<bool> m_dbvRunElevJogDown{"Elevator", "ElevJogDn", false};
  DashBoardValue<bool> m_dbvRunDeploManip{"Coral", "DeployManip", false};
  DashBoardValue<bool> m_dbvRunRetractManip{"Coral", "RetractManip", false};
  //DashBoardValue<bool> m_dbvRun{"", "", m_run};

  ESideSelected m_sideSelected = Unselected;

  frc2::InstantCommand m_toggleFieldRelative{[this] { m_fieldRelative = !m_fieldRelative; }, {}};
  frc2::InstantCommand m_toggleSlowSpeed{[this] { m_drive.ToggleSlowSpeed(); }, {}};
  frc2::InstantCommand m_setHighSpeedCmd{[this] { m_drive.SetSlowSpeed(false); }, {&m_drive}};

  frc2::InstantCommand m_resetOdo{[this] 
  {
    Pose2d tagPose;
    if (GetTagPose(tagPose))
    {
      Pose2d resetPose{m_drive.GetX(), m_drive.GetY(), tagPose.Rotation()};
      m_drive.ResetOdometry(resetPose);
    }
  }, {&m_drive}};
  
  frc2::InstantCommand m_elevL4{[this] { m_elevator.GoToPosition(L4); }, {&m_elevator} };
  frc2::InstantCommand m_elevL3{[this] { m_elevator.GoToPosition(L3); }, {&m_elevator} };
  frc2::InstantCommand m_elevL2{[this] { m_elevator.GoToPosition(L2); }, {&m_elevator} };
  frc2::InstantCommand m_elevL1{[this] { m_elevator.GoToPosition(L1); }, {&m_elevator} };

  frc2::InstantCommand m_elevL2_3{[this] { m_elevator.GoToPosition(algaeRemovalL2_3); }, {&m_elevator} };
  frc2::InstantCommand m_elevL3_4{[this] { m_elevator.GoToPosition(algaeRemovalL3_4); }, {&m_elevator} };

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
  //frc2::InstantCommand m_intakeParkForClimb{[this] { m_intake.ParkIntakeForClimb(); }, {&m_intake} };
  //frc2::InstantCommand m_intakeParkAtZero{[this] { m_intake.ParkIntakeAtZero(); }, {&m_intake} };

  frc2::InstantCommand m_coralStop{[this] { m_coral.Stop(); }, {&m_coral} };
  frc2::InstantCommand m_coralRetract{[this] { m_coral.RetractCoral(L1); }, {&m_coral} };
  frc2::InstantCommand m_coralDeployManip{[this] {m_coral.DeployManipulator(); }, {&m_coral} };
  frc2::InstantCommand m_coralRetractManip{[this] {m_coral.RetractManipulator(); }, {&m_coral} };

  frc2::InstantCommand m_rumblePrimary{[this] { m_primaryController.SetRumble(GenericHID::RumbleType::kBothRumble, 1); }, {} };
  frc2::InstantCommand m_stopRumblePrimary{[this] { m_primaryController.SetRumble(GenericHID::RumbleType::kBothRumble, 0); }, {} };

  frc2::InstantCommand m_ClimberDeployRelUp{[this] { m_climber.GoToPositionRel(c_defaultClimbDeployRelTurns);}, {&m_climber} };
  frc2::InstantCommand m_ClimberDeployRelDown{[this] { m_climber.GoToPositionRel(-c_defaultClimbDeployRelTurns);}, {&m_climber} };

  // For on the fly paths
//  PathConstraints m_pathConstraints { m_drive.m_currentMaxSpeed / 2.0, 4.0_mps_sq, 180_deg_per_s, 360_deg_per_s_sq };
  PathConstraints m_pathConstraints { 1.5_mps, 3.0_mps_sq, 180_deg_per_s, 360_deg_per_s_sq };
  std::shared_ptr<PathPlannerPath> m_path;

  frc::Timer m_timer; // For bringing the intake down after popping the pin
  Pose2d m_targetPose;

	wpi::log::DoubleLogEntry m_logRobotPoseX;
	wpi::log::DoubleLogEntry m_logRobotPoseY;
	wpi::log::DoubleLogEntry m_logRobotPoseRot;

	wpi::log::DoubleLogEntry m_logTargetPoseX;
	wpi::log::DoubleLogEntry m_logTargetPoseY;
	wpi::log::DoubleLogEntry m_logTargetPoseRot;

//#define TEST_WHEEL_CONTROL
#ifdef TEST_WHEEL_CONTROL
#define DISABLE_DRIVING
  frc2::InstantCommand m_wheelsForward{[this] { GetDrive().WheelsForward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsLeft{[this] { GetDrive().WheelsLeft(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsBackward{[this] { GetDrive().WheelsBackward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsRight{[this] { GetDrive().WheelsRight(); }, {&m_drive} };
#endif  // TEST_WHEEL_CONTROL
};
