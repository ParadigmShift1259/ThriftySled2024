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

RobotContainer* RobotContainer::m_pThis = nullptr;

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

  frc::SmartDashboard::PutNumber("InitPose", 180.0);

  if (!AutoBuilder::isConfigured())
  {
      AutoBuilder::configure(
          [this]() { return m_drive.GetPose(); }, // Function to supply current robot pose
          [this](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
          [this]() { return m_drive.GetChassisSpeeds(); },
          [this](frc::ChassisSpeeds speeds) { m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false); }, // Output function that accepts field relative ChassisSpeeds
          std::make_shared<PPHolonomicDriveController>(PIDConstants(5.0, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0)),
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

  // std::vector<frc::Pose2d> poses
  // {  
  //       frc::Pose2d ( 0_m, 0_m, frc::Rotation2d ( 0_deg ) )
  //     , frc::Pose2d ( 0_m, 0_m, frc::Rotation2d ( 0_deg ) )
  // };

  // //std::shared_ptr<PathPlannerPath> path = std::make_shared<PathPlannerPath>(
  // m_path = std::make_shared<PathPlannerPath>(
  //     PathPlannerPath::waypointsFromPoses(poses),
  //     m_pathConstraints,
  //     std::nullopt, // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
  //     GoalEndState(0.0_mps, frc::Rotation2d{ 0_deg } )
  // );

  SetDefaultCommands();
  ConfigureBindings();

  frc::SmartDashboard::PutNumber("elevHeight", 0.0);
  frc::SmartDashboard::PutNumber("elevDelay", 0.015);

  frc::SmartDashboard::PutBoolean("RightSelected", false);
  frc::SmartDashboard::PutBoolean("LeftSelected", false);
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return autos::ExampleAuto(&m_subsystem);
// }

void RobotContainer::Periodic()
{
  m_drive.Periodic();
  static int count = 0;
  if (count++ % 25 == 0)
  {
    RobotContainer::ConfigureRobotLEDs();
  }
  // m_dbvFieldRelative.Put(m_fieldRelative);
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

  primary.A().OnTrue(&m_coralEject);
  primary.B().OnTrue(CoralPrepCommand(*this, c_defaultL4Turns).ToPtr());
  primary.X().OnTrue(CoralIntakeCommand(*this).ToPtr());
  primary.Y().OnTrue(&m_coralStop);
  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.Start().OnTrue(&m_toggleSlowSpeed);
  primary.Back().OnTrue(&m_resetOdo);

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
    , ElevatorGoToCommand(*this, 2.0)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, 0.0)
  }.ToPtr());
  secondary.Y().OnTrue(&m_elevL4);

  secondary.LeftBumper().OnTrue(CoralEjectPostCommand(*this).ToPtr());
  secondary.RightBumper().OnTrue(frc2::SequentialCommandGroup{
    CoralPrepCommand(*this, c_defaultL4Turns)
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
    , ElevatorGoToCommand(*this, 2.0)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, 0.0)
    , m_elevReset
  }.ToPtr());
  secondary.Start().OnTrue(&m_coralRetract);
  secondary.POVDown().OnTrue(&m_elevRelPosDown);
  secondary.POVUp().OnTrue(&m_elevRelPosUp);
  secondary.LeftTrigger().OnTrue(&m_elevL3_4);
  secondary.RightTrigger().OnTrue(&m_elevL2_3);
  secondary.LeftStick().OnTrue(&m_intakeAlign);
  secondary.RightStick().OnTrue(&m_intakePark);
  secondary.POVRight().OnTrue(&m_coralDeployManip);
  secondary.POVLeft().OnTrue(&m_coralRetractManip);

  m_netButtonTest.OnChange(PrintCommand("Network button changed").ToPtr());

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
    , ElevatorGoToCommand(*this, 2.0)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, 0.0)
  }.ToPtr());
#else
  buttonBox.X().OnTrue(&m_setL4);
  buttonBox.Y().OnTrue(&m_setL3);
  buttonBox.RightBumper().OnTrue(&m_setL2);
  buttonBox.LeftBumper().OnTrue(&m_setL1);
#endif

  buttonBox.LeftTrigger().OnTrue(&m_setLeft);
  buttonBox.RightTrigger().OnTrue(&m_setRight);
  
  buttonBox.Back().OnTrue(&m_elevRelPosUp);
  buttonBox.LeftStick().OnTrue(&m_elevRelPosDown);

  buttonBox.B().OnTrue(&m_intakeParkAtZero);
  

  buttonBox.Start().OnTrue(&m_elevL3_4);
  buttonBox.RightStick().OnTrue(&m_elevL2_3);

  buttonBox.A().OnTrue(frc2::SequentialCommandGroup{
    CoralPrepCommand(*this, c_defaultL4Turns)
    , ConditionalCommand (InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} }, 
                          InstantCommand{[
                            this] {m_coral.RetractManipulator(); }, {&m_coral} }, [this](){return m_elevator.GetPresetLevel() == L4;})
    , WaitCommand(0.75_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
  }.ToPtr());

  //buttonBox.A().OnTrue(&m_coralRetract);

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
  constexpr double c_HolomonicP = 0.001;

  return FollowPathCommand(
        GetOnTheFlyPath()
      , [this]() { return m_drive.GetPose(); } // Function to supply current robot pose
      , [this]() { return m_drive.GetChassisSpeeds(); }
      , [this](const frc::ChassisSpeeds& speeds, const DriveFeedforwards &dffs) { m_drive.Drive(speeds, dffs); } // Output function that accepts field relative ChassisSpeeds
      , std::dynamic_pointer_cast<PathFollowingController>(std::make_shared<PPHolonomicDriveController>(PIDConstants(c_HolomonicP, 0.0, 0.0), PIDConstants(c_HolomonicP, 0.0, 0.0)))
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
}

const double c_tolerance = 0.02;
const double c_minInput = 0.07;

const units::length::meter_t c_halfRobotSize = 14.5_in + 3.0_in;  // Half robot width/length + bumper 29 / 2 = 14.5 + 3.0]
const units::length::meter_t c_reefPoleOffset = 6.5_in;  // reef poles are 13 inches apart, this is half
const double c_cosine30 = sqrt(3) * 0.5;

// Tag 10
const units::length::meter_t c_targetRedX = 481.39_in;
const units::length::meter_t c_targetRedY = 158.50_in;
const units::angle::degree_t c_targetRedRot = 0_deg;

const int c_tagIdReefRed = 10;
const double c_targetReefRedX = (c_targetRedX - c_halfRobotSize).value();
const double c_targetReefRedY = (c_targetRedY).value();
const double c_targetReefRedRot = c_targetRedRot.value();
const double c_targetReefRedOffsetX = 0.0;
const double c_targetReefRedOffsetY = c_reefPoleOffset.value() * 0.5;

const double c_targetLeftReefRedX = c_targetReefRedX - c_targetReefRedOffsetX;
const double c_targetLeftReefRedY = c_targetReefRedY - c_targetReefRedOffsetX;
const double c_targetRightReefRedX = c_targetReefRedX + c_targetReefRedOffsetX;
const double c_targetRightReefRedY = c_targetReefRedY + c_targetReefRedOffsetX;

std::shared_ptr<PathPlannerPath> RobotContainer::GetOnTheFlyPath()
{
  std::shared_ptr<PathPlannerPath> path;

  //path = PathPlannerPath::fromPathFile("Tag10Path");

  auto x = m_drive.GetX();
  auto y = m_drive.GetY();
  //auto rotation = m_drive.GetGyroAzimuthDeg().value();

  //const units::length::meter_t c_jogLength = 5.0_in;

  // int tagId = m_visionSubsystem.GetTagId();
  m_targetX = c_targetReefRedX;
  m_targetY = c_targetReefRedY;            
  m_targetRot = c_targetReefRedRot;

  frc::SmartDashboard::PutNumber("targetX", m_targetX);
  frc::SmartDashboard::PutNumber("targetY", m_targetY);
  frc::SmartDashboard::PutNumber("targetRot", m_targetRot);

  auto xM = units::length::meter_t {x};
  auto yM = units::length::meter_t {y};
  //auto rotationDeg = frc::Rotation2d {units::angle::degree_t {rotation}};

  auto xTarget = units::length::meter_t {m_targetX};
  auto yTarget = units::length::meter_t {m_targetY};

  // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html
  // Create a vector of waypoints from poses. Each pose represents one waypoint.
  // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
  // std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, rotationDeg }
  //                                 , frc::Pose2d { xTarget, yTarget, rotationTarget }
  // };
  //double tanAngle = -floor(180.0 + atan(yM.value() / xM.value()) * 180.0 / std::numbers::pi);

  double xDelta = xTarget.value() - xM.value();
  double yDelta = yTarget.value() - yM.value();
  double tanAngle = atan(yDelta / xDelta) * 180.0 / std::numbers::pi;
  printf("tanAngle %.3f\n", tanAngle);
  //frc::SmartDashboard::PutNumber("InitPose", tanAngle);
  //double dashAngle = frc::SmartDashboard::GetNumber("InitPose", tanAngle);
  std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, units::angle::degree_t{tanAngle} }
  //std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, units::angle::degree_t(dashAngle) }
  //std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, 180_deg }
  //std::vector<frc::Pose2d> poses {  frc::Pose2d { xM, yM, 0_deg }
                                  , frc::Pose2d { xTarget, yTarget, 0_deg }
  };

  path = std::make_shared<PathPlannerPath>(
      PathPlannerPath::waypointsFromPoses(poses),
      m_pathConstraints,
      std::nullopt, // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
      GoalEndState(0.0_mps, frc::Rotation2d{ 0_deg } )
      //GoalEndState(0.0_mps, frc::Rotation2d{ units::angle::degree_t{m_targetRot} } )
  );

  // Prevent the path from being flipped if the coordinates are already correct
  path->preventFlipping = true;

  //frc::SmartDashboard::PutData("On-the-fly path", path.get());

  printf("init heading %.3f\n", path->getInitialHeading().Degrees().value());
  printf("start holonom pose x %.3f y  %.3f rot  %.3f\n", path->getStartingHolonomicPose()->X().value(), path->getStartingHolonomicPose()->Y().value(), path->getStartingHolonomicPose()->Rotation().Degrees().value());

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

//  m_drive.ResetOdometry(path->getStartingHolonomicPose().value());

  return path;
}

void RobotContainer::ConfigureRobotLEDs()
{
#ifdef LED
  GetLED().Periodic();
#endif
}