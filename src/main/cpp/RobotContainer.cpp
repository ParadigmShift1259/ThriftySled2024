// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/GoToPositionCommand.h"
#include "commands/ElevatorGoToCommand.h"
#include "commands/CoralIntakeCommand.h"
#include "commands/CoralPrepCommand.h"
#include "commands/CoralEjectCommand.h"
#include "commands/CoralEjectPostCommand.h"
#include "commands/StopAllCommand.h"
#include "commands/ClimbRetractCommand.h"
#include "commands/ClimbDeployCommand.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DriverStation.h>

#include <cmath>

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/DeferredCommand.h>

#include <pathplanner/lib/commands/FollowPathCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPoint.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/smartdashboard/Field2d.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

frc::Field2d m_field;

RobotContainer* RobotContainer::m_pThis = nullptr;

constexpr double c_HolomonicTranslateP = 4.0;
constexpr double c_HolomonicRotateP = 1.5;

// Configure the button bindings
RobotContainer::RobotContainer() 
  : m_drive()
#ifdef USE_ORCESTRA
  , m_orchestra("output.chrp")
#endif
{
    m_pThis = this;
    //---------------------------------------------------------
    //printf("************************Calling SilenceJoystickConnectionWarning - Wisco2024 Day 1 only REMOVE!!!!!\n");
    DriverStation::SilenceJoystickConnectionWarning(true);
    //---------------------------------------------------------

    wpi::log::DataLog& log = GetLogger();

    m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/path/robotX");
    m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/path/robotY");
    m_logRobotPoseRot = wpi::log::DoubleLogEntry(log, "/path/robotRot");

    m_logTargetPoseX = wpi::log::DoubleLogEntry(log, "/path/targetX");
    m_logTargetPoseY = wpi::log::DoubleLogEntry(log, "/path/targetY");
    m_logTargetPoseRot = wpi::log::DoubleLogEntry(log, "/path/targetRot");

    frc::SmartDashboard::PutNumber("InitPose", 180.0);

    frc::SmartDashboard::PutData("Field", &m_field);
    
    // Logging callback for current robot pose
    PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose)
    {
        //printf("curr pose x %.3f y %.3f Rot %.3f\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());
        m_logRobotPoseX.Append(pose.X().value());
        m_logRobotPoseY.Append(pose.Y().value());
        m_logRobotPoseRot.Append(pose.Rotation().Degrees().value());

        m_field.SetRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose)
    {
        //printf("targ pose x %.3f y %.3f Rot %.3f\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());
        m_logTargetPoseX.Append(pose.X().value());
        m_logTargetPoseY.Append(pose.Y().value());
        m_logTargetPoseRot.Append(pose.Rotation().Degrees().value());
        m_field.GetObject("target pose")->SetPose(pose);
    });

    // Logging callback for the active path, this is sent as a vector of poses
    PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) 
    {
        // for (auto& pose : poses)
        // {
        //   printf("actv pose x %.3f y %.3f Rot %.3f\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());

        // }
        m_field.GetObject("path")->SetPoses(poses);
    });

  if (!AutoBuilder::isConfigured())
  {
      AutoBuilder::configure(
          [this]() { return m_drive.GetPose(); }, // Function to supply current robot pose
          [this](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
          [this]() { return m_drive.GetChassisSpeeds(); },
          [this](frc::ChassisSpeeds speeds) { m_drive.Drive(-speeds.vx, -speeds.vy, -speeds.omega, false); }, // Output function that accepts field relative ChassisSpeeds
          std::make_shared<PPHolonomicDriveController>(PIDConstants(c_HolomonicTranslateP, 0.0, 0.0), PIDConstants(c_HolomonicRotateP, 0.0, 0.0)),
          m_drive.GetRobotCfg(),
          [this]() 
          {
              auto alliance = frc::DriverStation::GetAlliance();
              if (alliance)
              {
                  return alliance.value() == frc::DriverStation::Alliance::kRed;
              }
              return false; 
          }, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          &m_drive // Drive requirements, usually just a single drive subsystem
      );
  }
  m_chooser = AutoBuilder::buildAutoChooser();	
  frc::SmartDashboard::PutData("Auto", &m_chooser);

  SetDefaultCommands();
  ConfigureBindings();

  frc::SmartDashboard::PutNumber("elevHeight", 0.0);
  frc::SmartDashboard::PutNumber("elevDelay", 0.015);

  frc::SmartDashboard::PutBoolean("RightSelected", false);
  frc::SmartDashboard::PutBoolean("LeftSelected", false);
  frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());
  SmartDashboard::PutNumber("PathTransP", c_HolomonicTranslateP);

  // frc2::NetworkButton
  // (
  //   nt::NetworkTableInstance::GetDefault().GetBooleanTopic(m_dbvFieldRelative.Path())).OnChange
  //   (
  //     frc2::cmd::RunOnce([this] { m_fieldRelative = !m_fieldRelative; }, {}
  //   )
  // );

  auto x = m_dbvRunIntakeStartup.Path();

  frc2::NetworkButton
  (
    nt::NetworkTableInstance::GetDefault().GetBooleanTopic(m_dbvRunIntakeStartup.Path())).OnChange
    (
      frc2::cmd::RunOnce([this] { if (m_runIntakeStartup) StartUp(); m_runIntakeStartup = false; }, {}
    )
  );
}

void RobotContainer::Periodic()
{
  if (m_timer.IsRunning() && m_timer.HasElapsed(0.75_s))
  {
    m_intake.ParkIntakeForLoad();
    m_timer.Stop();
  }

  static int count = 0;
  if (count++ % 25 == 0)
  {
    RobotContainer::ConfigureRobotLEDs();
  }
  
  m_dbvFieldRelative.Put(m_fieldRelative);
  m_dbvRunIntakeStartup.Put(m_runIntakeStartup);

  frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());
  m_field.SetRobotPose(m_drive.GetPose());
  frc::SmartDashboard::PutData("Field", &m_field);
}

void RobotContainer::SetDefaultCommands()
{
  m_drive.SetDefaultCommand(RunCommand
  (
    [this] 
    {
      // Don't send any input if autonomous is running
      if (m_isAutoRunning == false)
      {
        // const double kDeadband = 0.02;
        constexpr double kDeadband = 0.1;
		    constexpr double direction = 1.0;

        const auto xInput = direction* ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
        const auto yInput = direction * ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
        const auto rotInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);      
        // const auto rotXInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);
        // const auto rotYInput = ApplyDeadband(m_primaryController.GetRightY(), kDeadband);
        // const double rotX = m_rotLimiter.Calculate(rotXInput);
        // const double rotY = m_rotLimiter.Calculate(rotYInput);
        const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * m_drive.m_currentMaxSpeed;
        auto ySpeed = m_yspeedLimiter.Calculate(yInput) * m_drive.m_currentMaxSpeed;
        auto rot = m_rotLimiter.Calculate(rotInput) * m_drive.m_currentMaxAngularSpeed;      

//#define DISABLE_DRIVING
#ifndef DISABLE_DRIVING
        // if (m_fieldRelative)
        // {
        //   GetDrive().RotationDrive(xSpeed, ySpeed, rotX, rotY, m_fieldRelative);
        // }
        // else
        // {
          GetDrive().Drive(xSpeed, ySpeed, rot, m_fieldRelative);
        // }
#endif// DISABLE_DRIVING
      }
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings()
{
  ConfigPrimaryButtonBindings();
  ConfigSecondaryButtonBindings();
#ifdef USE_BUTTON_BOX
  ConfigButtonBoxBindings();
#endif
}

void RobotContainer::ConfigPrimaryButtonBindings()
{
  auto& primary = m_primaryController;
 
  // Primary
  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start

//#if 0
  primary.RightBumper().OnTrue(DeferredCommand(GetFollowPathCommand, {&m_drive} ).ToPtr());
//#endif

  primary.A().OnTrue(frc2::SequentialCommandGroup{
      m_setHighSpeedCmd
    , CoralEjectCommand(*this)
    , ElevatorGoToCommand(*this, L2)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L1)
  }.ToPtr());
  primary.B().OnTrue(CoralPrepCommand(*this, L4).ToPtr());
#define CLIMB
#ifdef CLIMB
  primary.X().OnTrue(&m_ClimberDeploy);
  primary.Back().OnTrue(ClimbRetractCommand(*this).ToPtr()); // Temp
#else  
  primary.X().OnTrue(CoralIntakeCommand(*this).ToPtr());
  primary.Back().OnTrue(ClimbDeployCommand(*this).ToPtr());
#endif

  primary.Y().OnTrue(&m_coralStop);
  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.Start().OnTrue(&m_toggleSlowSpeed);
//  primary.Back().OnTrue(&m_resetOdo);

  primary.POVUp().OnTrue(StopAllCommand(*this).ToPtr());
  primary.POVDown().OnTrue(InstantCommand([this]{GetOnTheFlyPath();},{&m_drive}).ToPtr());

//   primary.POVUp().OnTrue(GoToPositionCommand(*this, eJogForward, m_path).ToPtr());
//   primary.POVDown().OnTrue(GoToPositionCommand(*this, eJogBackward, m_path).ToPtr());
//   primary.POVRight().OnTrue(GoToPositionCommand(*this, eJogRight, m_path).ToPtr());
//   primary.POVLeft().OnTrue(GoToPositionCommand(*this, eJogLeft, m_path).ToPtr());
}

void RobotContainer::ConfigSecondaryButtonBindings()
{
  auto& secondary = m_secondaryController;

  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
  secondary.A().OnTrue(&m_elevL2);
  secondary.B().OnTrue(&m_elevL3);
  secondary.X().OnTrue(frc2::SequentialCommandGroup{
    m_setHighSpeedCmd
    , ElevatorGoToCommand(*this, L2)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L1)
  }.ToPtr());
  secondary.Y().OnTrue(&m_elevL4);

  secondary.LeftBumper().OnTrue(CoralEjectPostCommand(*this).ToPtr());
  secondary.RightBumper().OnTrue(frc2::SequentialCommandGroup{
    CoralPrepCommand(*this, L4)
    , ConditionalCommand (InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} }, 
                          InstantCommand{[this] {m_coral.RetractManipulator(); }, {&m_coral} }, [](){return true;})
    , WaitCommand(0.75_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
  }.ToPtr());
      
  // secondary.Back().OnTrue(&m_elevReset);
  secondary.Back().OnTrue(frc2::SequentialCommandGroup{
    m_setHighSpeedCmd
    , ElevatorGoToCommand(*this, L2)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L2)
    , m_elevReset
  }.ToPtr());
  secondary.Start().OnTrue(&m_coralRetract);
  secondary.POVDown().OnTrue(&m_elevRelPosDown);
  secondary.POVUp().OnTrue(&m_elevRelPosUp);
  secondary.LeftTrigger().OnTrue(&m_elevL3_4);
  secondary.RightTrigger().OnTrue(&m_elevL2_3);
  secondary.LeftStick().OnTrue(&m_intakeAlign);
  secondary.RightStick().OnTrue(&m_intakeParkForClimb);
  secondary.POVRight().OnTrue(&m_coralDeployManip);
  secondary.POVLeft().OnTrue(&m_coralRetractManip);

  //m_netButtonTest.OnChange(PrintCommand("Network button changed").ToPtr());

#ifdef TEST_WHEEL_CONTROL
  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVUp(loop).Rising().IfHigh([this] { m_wheelsLeft.Schedule(); });
  secondary.POVRight(loop).Rising().IfHigh([this] { m_wheelsForward.Schedule(); });
#endif  //  TEST_WHEEL_CONTROL
}

#ifdef USE_BUTTON_BOX
void RobotContainer::ConfigButtonBoxBindings()
{
  auto& buttonBox = m_buttonController;

  // Physical layout and XBox assignment
  // ┌───────┐       ┌───────┬───────┐
  // │Green1 │       │White2 │ Blue2 │
  // │  X    │       │  Back │ Start │
  // ├───────┤       ├───────┼───────┤
  // │Yellow1│       │Green2 │ Red2  │
  // │  Y    │       │  LS   │  RS   │
  // ├───────┤       ├───────┼───────┤
  // │Blue1  │       │Black2 │Yellow2│
  // │  RB   │       │  B    │  A    │
  // ├───────┼───────┼───────┤───────┘
  // │Black1 │White1 │ Red1  │        
  // │  LB   │   LT  │  RT   │        
  // └───────┴───────┴───────┘  
#ifdef PRACTICE_BINDINGS           
  buttonBox.X().OnTrue(&m_elevL4);
  buttonBox.Y().OnTrue(&m_elevL3);
  buttonBox.RightBumper().OnTrue(&m_elevL2);
  buttonBox.LeftBumper().OnTrue(frc2::SequentialCommandGroup{
    m_setHighSpeedCmd
    , ElevatorGoToCommand(*this, L2)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L1)
  }.ToPtr());
#else
  buttonBox.X().OnTrue(&m_setL4);
  buttonBox.Y().OnTrue(&m_setL3);
  buttonBox.RightBumper().OnTrue(&m_setL2);
  buttonBox.LeftBumper().OnTrue(&m_setL1);
#endif

  // buttonBox.LeftTrigger().OnTrue(&m_setLeft);
  // buttonBox.RightTrigger().OnTrue(&m_setRight);
  buttonBox.LeftTrigger().OnTrue(frc2::SequentialCommandGroup{
    InstantCommand{[this](){m_drive.WheelsForward();}, {&m_drive}}
    , WaitCommand(0.125_s)
    , InstantCommand{[this](){m_drive.Stop();}, {&m_drive}}
  }.ToPtr());
//  buttonBox.RightTrigger().OnTrue(CoralIntakeCommand(*this).ToPtr());
  
  buttonBox.Back().OnTrue(&m_elevRelPosUp);
  
  buttonBox.LeftStick().OnTrue(&m_elevRelPosDown);

  //buttonBox.B().OnTrue(&m_intakeParkAtZero);
  //buttonBox.B().OnTrue(&m_intakeAlign);
  buttonBox.B().OnTrue(frc2::SequentialCommandGroup{
      CoralIntakeCommand(*this)
    , m_setL3
    , m_elevL3
  }.ToPtr());
  buttonBox.Start().OnTrue(&m_elevL3_4);
  buttonBox.RightStick().OnTrue(&m_elevL2_3);

  buttonBox.A().OnTrue(frc2::SequentialCommandGroup{
      CoralPrepCommand(*this, L4)
    , ConditionalCommand (InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} }, 
                          InstantCommand{[
                            this] {m_coral.RetractManipulator(); }, {&m_coral} }, [this](){return m_elevator.GetPresetLevel() == L4;})
    , WaitCommand(0.75_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
  }.ToPtr());

  //buttonBox.A().OnTrue(&m_coralRetract);

  buttonBox.POVUp().OnTrue(&m_ClimberDeployRelUp);
  buttonBox.POVDown().OnTrue(&m_ClimberDeployRelDown);

  // Wiring
  // See Game Pad 2040 project https://gp2040-ce.info/introduction
  //
  //        │             │                          │           │      
  //        │             │ GP                    GP │           │      
  // Button │    XBox     │2040    RasPI Pico    2040│   XBox    │Button
  //        │             │    ┌────────────────┐    │           │      
  // Yellow2│            A│ B1 │GP06            │    │           │      
  // Black2 │            B│ B2 │GP07            │    │           │      
  // Red1   │   Right Trig│ R2 │GP08            │    │           │      
  // White1 │    Left Trig│ L2 │GP09            │    │           │      
  //        │             │    │GND          GND│    │           │      
  // Green1 │            X│ B3 │GP10        GP21│ A2 │           │      
  // Yellow1│            Y│ B4 │GP11        GP20│ A1 │           │      
  // Blue1  │ Right Bumper│ R1 │GP12        GP19│ R3 │Right Stick│Red2  
  // Black1 │  Left Bumper│ L1 │GP13        GP18│ L3 │Left Stick │Green2
  //                           │GND          GND│    │           │      
  //                           │            GP17│ S2 │Start      │Blue2 
  //                           │            GP16│ S1 │Back       │White2
  //                           └────────────────┘                       
}
#endif

Command* RobotContainer::GetAutonomousCommand()
{
  printf("auto chosen %s\n", m_chooser.GetSelected()->GetName().c_str());
  //return m_chooser.GetSelected();
  PathPlannerAuto* pPpa = (PathPlannerAuto*)m_chooser.GetSelected();
  //pPpa->activePath().
  return pPpa;
}

void RobotContainer::StopAll()
{
  m_elevator.Stop();
}

void RobotContainer::SetSideSelected(ESideSelected sideSelected)
{
  m_sideSelected = sideSelected;
  frc::SmartDashboard::PutBoolean("RightSelected", (m_sideSelected == RightSide));
  frc::SmartDashboard::PutBoolean("LeftSelected", (m_sideSelected == LeftSide));
}

frc2::CommandPtr RobotContainer::GetFollowPathCommandImpl()
{
//#define STOP_AFTER_OTF_PATH
#ifdef STOP_AFTER_OTF_PATH
   return frc2::SequentialCommandGroup{
      FollowPathCommand(
        GetOnTheFlyPath()
      , [this]() { return m_drive.GetPose(); } // Function to supply current robot pose
      , [this]() { return m_drive.GetChassisSpeeds(); }
      , [this](const frc::ChassisSpeeds& speeds, const DriveFeedforwards &dffs) { m_drive.Drive(speeds, dffs); } // Output function that accepts field relative ChassisSpeeds
      , std::dynamic_pointer_cast<PathFollowingController>(std::make_shared<PPHolonomicDriveController>(PIDConstants(c_HolomonicTranslateP, 0.0, 0.0), PIDConstants(c_HolomonicRotateP, 0.0, 0.0)))
      , m_drive.GetRobotCfg()
      , [this]() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            auto alliance = DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == DriverStation::Alliance::kRed;
            }
            return false;
        },
        {&m_drive} // Drive requirements, usually just a single drive subsystem
      )
    , InstantCommand([this] (){ m_drive.Stop(); }, {&m_drive})
  }.ToPtr();
#else
  auto p = SmartDashboard::GetNumber("PathTransP", c_HolomonicTranslateP);
  return FollowPathCommand(
        GetOnTheFlyPath()
      , [this]() { return m_drive.GetPose(); } // Function to supply current robot pose
      , [this]() { return m_drive.GetChassisSpeeds(); }
      , [this](const frc::ChassisSpeeds& speeds, const DriveFeedforwards &dffs) { m_drive.Drive(speeds, dffs); } // Output function that accepts field relative ChassisSpeeds
      , std::dynamic_pointer_cast<PathFollowingController>(std::make_shared<PPHolonomicDriveController>(PIDConstants(p, 0.0, 0.0), PIDConstants(c_HolomonicRotateP, 0.0, 0.0)))
      , m_drive.GetRobotCfg()
      , [this]() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            auto alliance = DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == DriverStation::Alliance::kRed;
            }
            return false;
        },
        {&m_drive} // Drive requirements, usually just a single drive subsystem
      ).ToPtr();
#endif  // STOP_AFTER_OTF_PATH
}

// const units::length::meter_t c_halfRobotSize = 14.5_in + 3.0_in;  // Half robot width/length + bumper 29 / 2 = 14.5 + 3.0]
// const units::length::meter_t c_reefPoleOffset = 6.5_in;  // reef poles are 13 inches apart, this is half
// const double c_cosine30 = sqrt(3) * 0.5;

// Tag 10
// const units::length::meter_t c_targetRedX = 481.39_in;
// const units::length::meter_t c_targetRedY = 158.50_in;
// const units::angle::degree_t c_targetRedRot = 0_deg;

// const int c_tagIdReefRed = 10;
// const double c_targetReefRedX = (c_targetRedX - c_halfRobotSize).value();
// const double c_targetReefRedY = (c_targetRedY).value();
// const double c_targetReefRedRot = c_targetRedRot.value();
// const double c_targetReefRedOffsetX = 0.0;
// const double c_targetReefRedOffsetY = c_reefPoleOffset.value() * 0.5;

// const double c_targetLeftReefRedX = c_targetReefRedX - c_targetReefRedOffsetX;
// const double c_targetLeftReefRedY = c_targetReefRedY - c_targetReefRedOffsetX;
// const double c_targetRightReefRedX = c_targetReefRedX + c_targetReefRedOffsetX;
// const double c_targetRightReefRedY = c_targetReefRedY + c_targetReefRedOffsetX;

const std::map<int, frc::Pose2d> c_mapTagPoses
{
    {  6, frc::Pose2d { 530.49_in, 130.17_in, 120_deg } }
  , { 10, frc::Pose2d { 481.39_in, 158.5_in,    0_deg } }
};

std::shared_ptr<PathPlannerPath> RobotContainer::GetOnTheFlyPath()
{
  std::shared_ptr<PathPlannerPath> path;

  auto currentX = m_drive.GetX();
  auto currentY = m_drive.GetY();

  units::length::meter_t targetX;
  units::length::meter_t targetY;
  units::angle::degree_t targetRot;

  int tagId = m_vision.GetTagId();
  // targetX = c_targetReefRedX - 0.125;  // Added for test
  // targetY = c_targetReefRedY;            
  if (tagId != -1)
  {
    auto it = c_mapTagPoses.find(tagId);
    if (it != c_mapTagPoses.end())
    {
      targetX = it->second.X() - 0.125;  // Added for test
      targetY = it->second.Y();
      targetRot = it->second.Rotation().Degrees();
    }
  }

  frc::SmartDashboard::PutNumber("targetX", targetX.value());
  frc::SmartDashboard::PutNumber("targetY", targetY.value());
  frc::SmartDashboard::PutNumber("targetRot", targetRot.value());

  double xDelta = targetX.value() - currentX;
  double yDelta = targetY.value() - currentY;
  double tanAngle = atan(yDelta / xDelta) * 180.0 / std::numbers::pi;
  printf("tanAngle %.3f\n", tanAngle);
  std::vector<frc::Pose2d> poses {  frc::Pose2d { currentX, currentY, units::angle::degree_t{tanAngle} }
  //std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, units::angle::degree_t(dashAngle) }
  //std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, 180_deg }
  //std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, 0_deg }
                                  , frc::Pose2d { targetX, targetY, targetRot }
  };

  path = std::make_shared<PathPlannerPath>(
      PathPlannerPath::waypointsFromPoses(poses),
      m_pathConstraints,
      std::nullopt, // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
      //GoalEndState(0.0_mps, frc::Rotation2d{ 0_deg } )// zero swerve rotation angle for tag 10
      GoalEndState(0.0_mps, frc::Rotation2d{ targetRot } )
  );

  // Prevent the path from being flipped if the coordinates are already correct
  path->preventFlipping = true;

  printf("init heading %.3f\n", path->getInitialHeading().Degrees().value());
  //printf("start holonom pose x %.3f y  %.3f rot  %.3f\n", path->getStartingHolonomicPose()->X().value(), path->getStartingHolonomicPose()->Y().value(), path->getStartingHolonomicPose()->Rotation().Degrees().value());

  printf("input path poses x y angle\n");
  for (auto& wp : poses)
  {
      printf("%.3f    %.3f    %.3f\n", wp.X().value(), wp.Y().value(), wp.Rotation().Degrees().value());
  }

  printf("path poses x y angle\n");
  for (auto& wp : path->getPathPoses())
  {
      printf("%.3f    %.3f    %.3f\n", wp.X().value(), wp.Y().value(), wp.Rotation().Degrees().value());
  }

  //RotationTarget rt(0.1, 0_deg);
  //path->getRotationTargets().push_back(rt);

  printf("rot targ pos targ\n");
  for (auto& rt : path->getRotationTargets())
  {
      printf("%.3f    %.3f\n", rt.getPosition(), rt.getTarget().Degrees().value());
  }

  printf("path points x y angle\n");
  auto pts = path->getAllPathPoints();
  for (auto& pt : pts)
  {
      printf("%.3f    %.3f    %.3f\n", pt.position.X().value(), pt.position.Y().value(), pt.position.Angle().Degrees().value());
  }

  // Do not need to reset odemettry for one the fly path

  return path;
}

void RobotContainer::ConfigureRobotLEDs()
{
#ifdef LED
  GetLED().Periodic();
#endif
}