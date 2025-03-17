// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"

#include "commands/ElevatorGoToCommand.h"
#include "commands/CoralIntakeCommand.h"
#include "commands/CoralPrepCommand.h"
#include "commands/CoralEjectCommand.h"
#include "commands/CoralEjectPostCommand.h"
#include "commands/StopAllCommand.h"
#include "commands/ClimbRetractCommand.h"
#include "commands/ClimbDeployCommand.h"
#include "commands/PositionPIDCommand.h"
#include "commands/ResetAngleAtLoadCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/PrintCommand.h>
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
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <cmath>

frc::Field2d m_field;

RobotContainer* RobotContainer::m_pThis = nullptr;

// Data for each tag position
struct TagInfo
{
  TagInfo(double targetRot
        , double leftX
        , double leftY
        , double midX
        , double midY
        , double rightX
        , double rightY)
    : m_poseLeft(units::length::meter_t{leftX}, units::length::meter_t{leftY}, units::angle::degree_t{targetRot})
    , m_poseMid(units::length::meter_t{midX}, units::length::meter_t{midY}, units::angle::degree_t{targetRot})
    , m_poseRight(units::length::meter_t{rightX}, units::length::meter_t{rightY}, units::angle::degree_t{targetRot})
  {
  }

  Pose2d m_poseLeft;
  Pose2d m_poseMid;
  Pose2d m_poseRight;

  Pose2d& GetPose(ESideSelected side)
  {
    if (side == LeftSide)
    {
      return m_poseLeft;
    }
    else if (side == RightSide)
    {
      return m_poseRight;
    }
    
    return m_poseMid;
  }
};

 //ID X Y Z Z-Rotation Y-Rotation
//  1 657.37 25.80 58.50 126 0
//  2 657.37 291.20 58.50 234 0
//  3 455.15 317.15 51.25 270 0
//  4 365.20 241.64 73.54 0 30
//  5 365.20 75.39 73.54 0 30
//  12 33.51 25.80 58.50 54 0
//  13 33.51 291.20 58.50 306 0
//  14 325.68 241.64 73.54 180 30
//  15 325.68 75.39 73.54 180 30
//  16 235.73-0.15 51.25 90 0
std::map<int, Pose2d> c_mapTagLocations
{
    {  6, { 530.49_in, 130.17_in, 300_deg } }
  , {  7, { 546.87_in, 158.50_in,   0_deg } }
  , {  8, { 530.49_in, 186.83_in,  60_deg } }
  , {  9, { 497.77_in, 186.83_in, 120_deg } }
  , { 10, { 481.39_in, 158.50_in, 180_deg } }
  , { 11, { 497.77_in, 130.17_in, 240_deg } }
  , { 17, { 160.39_in, 130.17_in, 240_deg } }
  , { 18, { 144.00_in, 158.50_in, 180_deg } }
  , { 19, { 160.39_in, 186.83_in, 120_deg } }
  , { 20, { 193.10_in, 186.83_in,  60_deg } }
  , { 21, { 209.49_in, 158.50_in,   0_deg } }
  , { 22, { 193.10_in, 130.17_in, 300_deg } } 
};

// Calculated target poses in spreadsheet given the April tag locations in sheet 4 of https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
// ID	      X	   Y    Z-Rot  AdjAng      Sin         Cos         DeltaX        DeltaY        MidX MidY ReefOffsetX    ReefOffsetY    LeftX    LeftY    RightX   RightY
//  6 to 11	TagX TagY TagRot TagRot + 90 sin(AdjAng) cos(AdjAng) 22sin(AdjAng) 22cos(AdjAng) X+dX Y+dy 6.5cos(AdjAng) 6.5sin(AdjAng) MidX-ROX MidY+ROY MidX+ROX MidY-ROY
//    or
// 17 to 22

std::map<int, TagInfo> c_mapTagPoses;
#if 0
{
  // Offset from tag position 27.5 inches 
  // Tag ID   Left Pose                          Middle Pose                        Right Pose
    {  6, { { 538.61_in, 103.10_in, 120_deg }, { 544.24_in, 106.35_in, 120_deg }, { 549.87_in, 109.60_in, 120_deg } } }
  , {  7, { { 574.37_in, 152.00_in, 180_deg }, { 574.37_in, 158.50_in, 180_deg }, { 574.37_in, 165.00_in, 180_deg } } }
  , {  8, { { 549.87_in, 207.40_in, 240_deg }, { 544.24_in, 210.65_in, 240_deg }, { 538.61_in, 213.90_in, 240_deg } } }
  , {  9, { { 489.65_in, 213.90_in, 300_deg }, { 484.02_in, 210.65_in, 300_deg }, { 478.39_in, 207.40_in, 300_deg } } }
  , { 10, { { 453.89_in, 165.00_in,   0_deg }, { 453.89_in, 158.50_in,   0_deg }, { 453.89_in, 152.00_in,   0_deg } } }
  , { 11, { { 478.39_in, 109.60_in,  60_deg }, { 484.02_in, 106.35_in,  60_deg }, { 489.65_in, 103.10_in,  60_deg } } }
  , { 17, { { 141.01_in, 109.60_in,  60_deg }, { 146.64_in, 106.35_in,  60_deg }, { 152.27_in, 514.12_in,  60_deg } } }
  , { 18, { { 116.50_in, 165.00_in,   0_deg }, { 116.50_in, 158.50_in,   0_deg }, { 116.50_in, 152.00_in,   0_deg } } }
  , { 19, { { 152.27_in, 213.90_in, 300_deg }, { 146.64_in, 210.65_in, 300_deg }, { 141.01_in, 207.40_in, 300_deg } } }
  , { 20, { { 212.48_in, 207.40_in, 240_deg }, { 206.85_in, 210.65_in, 240_deg }, { 201.22_in, 213.90_in, 240_deg } } }
  , { 21, { { 236.99_in, 152.00_in, 180_deg }, { 236.99_in, 158.50_in, 180_deg }, { 236.99_in, 165.00_in, 180_deg } } }
  , { 22, { { 201.22_in, 103.10_in, 120_deg }, { 206.85_in, 106.35_in, 120_deg }, { 212.48_in, 109.60_in, 120_deg } } }

//    {  6, { { 541.49_in, 111.17_in, 120_deg }, { 541.49_in, 111.17_in, 120_deg }, { 541.49_in, 111.17_in, 120_deg } } }
//  , { 10, { { 459.39_in, 158.5_in,    0_deg }, { 541.49_in, 111.17_in, 120_deg }, { 541.49_in, 111.17_in, 120_deg } } }
    {  6, { { 535.86_in, 107.87_in, 120_deg }, { 541.49_in, 111.12_in, 120_deg }, { 547.12_in, 114.37_in, 120_deg } } }
  , {  7, { { 568.87_in, 152.00_in, 180_deg }, { 568.87_in, 158.50_in, 180_deg }, { 568.87_in, 165.00_in, 180_deg } } }
  , {  8, { { 547.12_in, 202.63_in, 240_deg }, { 541.49_in, 205.88_in, 240_deg }, { 535.86_in, 209.13_in, 240_deg } } }
  , {  9, { { 492.40_in, 209.13_in, 300_deg }, { 486.77_in, 205.88_in, 300_deg }, { 481.14_in, 202.63_in, 300_deg } } }
  , { 10, { { 453.89_in, 165.00_in,   0_deg }, { 453.80_in, 158.50_in,   0_deg }, { 453.89_in, 152.00_in,   0_deg } } }
//  , { 10, { { 459.39_in, 165.00_in,   0_deg }, { 459.39_in, 158.50_in,   0_deg }, { 459.39_in, 152.00_in,   0_deg } } }
  , { 11, { { 481.14_in, 114.37_in,  60_deg }, { 484.27_in, 106.79_in,  60_deg }, { 492.40_in, 107.87_in,  60_deg } } }
//  , { 11, { { 481.14_in, 114.37_in,  60_deg }, { 486.77_in, 111.12_in,  60_deg }, { 492.40_in, 107.87_in,  60_deg } } }
  , { 17, { { 143.76_in, 114.37_in,  60_deg }, { 149.39_in, 111.12_in,  60_deg }, { 155.02_in, 107.87_in,  60_deg } } }
  , { 18, { { 122.00_in, 165.00_in,   0_deg }, { 122.00_in, 158.50_in,   0_deg }, { 122.00_in, 152.00_in,   0_deg } } }
  , { 19, { { 155.02_in, 209.13_in, 300_deg }, { 149.39_in, 205.88_in, 300_deg }, { 143.76_in, 202.63_in, 300_deg } } }
  , { 20, { { 209.73_in, 202.63_in, 240_deg }, { 204.10_in, 205.88_in, 240_deg }, { 198.47_in, 209.13_in, 240_deg } } }
  , { 21, { { 231.49_in, 152.00_in, 180_deg }, { 231.49_in, 158.50_in, 180_deg }, { 231.49_in, 165.00_in, 180_deg } } }
  , { 22, { { 198.47_in, 107.87_in, 120_deg }, { 204.10_in, 111.12_in, 120_deg }, { 209.73_in, 114.37_in, 120_deg } } }
};
#endif

// Configure the button bindings
RobotContainer::RobotContainer() 
  : m_drive()
#ifdef USE_ORCESTRA
  , m_orchestra("output.chrp")
#endif
{
  commandRunning = false;
  NamedCommands::registerCommand("RaiseL3", std::move(frc2::SequentialCommandGroup{m_setL3, m_elevL3}.ToPtr()));
  NamedCommands::registerCommand("RaiseL2", std::move(frc2::SequentialCommandGroup{m_setL2, m_elevL2}.ToPtr()));

  NamedCommands::registerCommand("PlaceL2", std::move(
  frc2::SequentialCommandGroup{
    m_setL2
  , CoralPrepCommand(*this)
  , WaitCommand(0.75_s)
  , CoralEjectCommand(*this)
  , WaitCommand(0.25_s)
  , CoralEjectPostCommand(*this)
  }.ToPtr()));

  NamedCommands::registerCommand("ShootCoral", std::move(
  frc2::SequentialCommandGroup{
    CoralEjectCommand(*this)
  , m_elevL1
  }.ToPtr()));
  NamedCommands::registerCommand("StopCoral", std::move(
  frc2::SequentialCommandGroup{
    m_coralStop
  }.ToPtr()));

  NamedCommands::registerCommand("PlaceL4L_OTFP", std::move(
    frc2::SequentialCommandGroup{
      m_FollowPathLED
    ,  m_setL4
    , m_setLeft
    , CoralPrepCommand(*this)
    , DeferredCommand(GetFollowPathCommand, {&m_drive} )
    , m_setHighSpeedCmd
    , ConditionalCommand (SequentialCommandGroup{
                                //WaitCommand(0.20_s)
                                ElevatorGoToCommand(*this, L4, c_bUsePresetLevel)
                              , InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} } }, 
                          InstantCommand{[this] {m_coral.RetractManipulator(); }, {&m_coral} }, 
                          [this](){return (m_elevator.GetPresetLevel() == L4 || m_elevator.GetPresetLevel() == L1);})
    , WaitCommand(0.55_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
    , m_elevL1
    , m_EndLED
    }.ToPtr()));

  NamedCommands::registerCommand("PlaceL4R_OTFP", std::move(
    frc2::SequentialCommandGroup{
      m_FollowPathLED
    , m_setL4
    , m_setRight
    , CoralPrepCommand(*this)
    , DeferredCommand(GetFollowPathCommand, {&m_drive} )
    , m_setHighSpeedCmd
    , ConditionalCommand (SequentialCommandGroup{
                                //WaitCommand(0.20_s)
                                ElevatorGoToCommand(*this, L4, c_bUsePresetLevel)
                              , InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} } }, 
                          InstantCommand{[this] {m_coral.RetractManipulator(); }, {&m_coral} }, 
                          [this](){return (m_elevator.GetPresetLevel() == L4 || m_elevator.GetPresetLevel() == L1);})
    , WaitCommand(0.55_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
    , m_elevL1
    , m_EndLED
    }.ToPtr()));

  NamedCommands::registerCommand("Intake", std::move(
    frc2::SequentialCommandGroup{
       CoralIntakeCommand(*this)
      , m_elevL3
    }.ToPtr()));
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
          std::make_shared<PPHolonomicDriveController>(PIDConstants(DriveConstants::c_HolomonicTranslateP, 0.0, 0.0), 
                                                       PIDConstants(DriveConstants::c_HolomonicRotateP, 0.0, 0.0)),
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
  SmartDashboard::PutNumber("PathTransP", DriveConstants::c_HolomonicTranslateP);

  ConfigureNetworkButtons();
  CalcTargetPoses();
  PrintTargetPoses();
}

void RobotContainer::ConfigureNetworkButtons()
{
  using NetBtn = frc2::NetworkButton;
  using InstCmd = frc2::InstantCommand;

  auto netTable = nt::NetworkTableInstance::GetDefault();

  NetBtn(netTable.GetBooleanTopic(m_dbvRunIntakeStartup.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunIntakeStartup.Get()) StartUp(); m_dbvRunIntakeStartup.Put(false); }, {}).ToPtr()
  );

  NetBtn(netTable.GetBooleanTopic(m_dbvRunCoralRetract.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunCoralRetract.Get()) m_coral.RetractCoral(L1); m_dbvRunCoralRetract.Put(false); }, {}).ToPtr()
  );

  NetBtn(netTable.GetBooleanTopic(m_dbvRunElevL2.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunElevL2.Get()) m_elevator.GoToPosition(L2); m_dbvRunElevL2.Put(false); }, {}).ToPtr()
  );

  NetBtn(netTable.GetBooleanTopic(m_dbvRunElevL3.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunElevL3.Get()) m_elevator.GoToPosition(L3); m_dbvRunElevL3.Put(false); }, {}).ToPtr()
  );

  NetBtn(netTable.GetBooleanTopic(m_dbvRunElevL4.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunElevL4.Get()) m_elevator.GoToPosition(L4); m_dbvRunElevL4.Put(false); }, {}).ToPtr()
  );

  NetBtn(netTable.GetBooleanTopic(m_dbvRunElevJogDown.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunElevJogDown.Get()) m_elevator.GotoPositionRel(-1.0); m_dbvRunElevJogDown.Put(false); }, {}).ToPtr()
  );

  NetBtn(netTable.GetBooleanTopic(m_dbvRunDeploManip.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunDeploManip.Get()) m_coral.DeployManipulator(); m_dbvRunDeploManip.Put(false); }, {}).ToPtr()
  );

  NetBtn(netTable.GetBooleanTopic(m_dbvRunRetractManip.Path())).OnChange
  (
    InstCmd([this] { if (m_dbvRunRetractManip.Get()) m_coral.RetractManipulator(); m_dbvRunRetractManip.Put(false); }, {}).ToPtr()
  );

  // NetBtn
  // (
  //   netTable.GetBooleanTopic(m_dbvFieldRelative.Path())).OnChange
  //   (
  //     frc2::cmd::RunOnce([this] { m_fieldRelative = !m_fieldRelative; }, {}
  //   )
  // );
}

void RobotContainer::CalcTargetPoses()
{
  constexpr units::length::meter_t c_reefOffset = 25.0_in;
  constexpr units::length::meter_t c_leftRightOffset = 6.5_in;

  for (auto tagLoc : c_mapTagLocations)
  {
    int tagId = tagLoc.first;
    auto& tagPose = tagLoc.second;

    auto thetaDeg = fmod((tagPose.Rotation().Degrees().value() + 90.0), 360.0);
    double thetaRads = thetaDeg * std::numbers::pi / 180.0;
 
    double midX = tagPose.X().value() + c_reefOffset.value() * sin(thetaRads);
    double midY = tagPose.Y().value() - c_reefOffset.value() * cos(thetaRads);
    double targetRot = fmod((tagPose.Rotation().Degrees().value() + 180.0), 360.0);
    
    // Note order of x/y and sin/cos reversed from previous calculation
    double leftRightOffsetX = c_leftRightOffset.value() * cos(thetaRads);
    double leftRightOffsetY = -c_leftRightOffset.value() * sin(thetaRads);
    
    double leftX = midX - leftRightOffsetX;
    double leftY = midY + leftRightOffsetY;
    
    double rightX = midX + leftRightOffsetX;
    double rightY = midY - leftRightOffsetY;

    TagInfo ti(targetRot, leftX, leftY, midX, midY, rightX, rightY);
    c_mapTagPoses.emplace(std::make_pair(tagId, ti));
  }
}

void RobotContainer::PrintTargetPoses()
{
  for (auto pair : c_mapTagPoses)
  {
    int tagId = pair.first;
    auto& ti = pair.second;

    printf("In meters Tag %d ", tagId);
    printf(" left %.2f, %.2f, %.2f ", ti.m_poseLeft.X().value(), ti.m_poseLeft.Y().value(), ti.m_poseLeft.Rotation().Degrees().value());
    printf("  mid %.2f, %.2f, %.2f ", ti.m_poseMid.X().value(), ti.m_poseMid.Y().value(), ti.m_poseMid.Rotation().Degrees().value());
    printf("right %.2f, %.2f, %.2f\n", ti.m_poseRight.X().value(), ti.m_poseRight.Y().value(), ti.m_poseRight.Rotation().Degrees().value());
  }

  printf("\n");
  for (auto pair : c_mapTagPoses)
  {
    int tagId = pair.first;
    auto& ti = pair.second;

    printf("In inches Tag %d ", tagId);
    units::length::inch_t lx = ti.m_poseLeft.X();
    units::length::inch_t ly = ti.m_poseLeft.Y();
    printf(" left %.2f, %.2f, %.2f ", lx.value(), ly.value(), ti.m_poseLeft.Rotation().Degrees().value());
    units::length::inch_t mx = ti.m_poseMid.X();
    units::length::inch_t my = ti.m_poseMid.Y();
    printf("  mid %.2f, %.2f, %.2f ", mx.value(), my.value(), ti.m_poseMid.Rotation().Degrees().value());
    units::length::inch_t rx = ti.m_poseRight.X();
    units::length::inch_t ry = ti.m_poseRight.Y();
    printf("right %.2f, %.2f, %.2f\n", rx.value(), ry.value(), ti.m_poseRight.Rotation().Degrees().value());
  }
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
    ConfigureRobotLEDs();
  }

  m_dbvFieldRelative.Put(m_fieldRelative);

  frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());
  m_field.SetRobotPose(m_drive.GetPose());
  frc::SmartDashboard::PutData("Field", &m_field);

  AreWeInTheSweetSpot();
}

void RobotContainer::AreWeInTheSweetSpot()
{
  Pose2d targetPose;
  if (GetTagPose(targetPose))
  {
    auto targetX = targetPose.X();
    auto targetY = targetPose.Y();
    auto targetRot = targetPose.Rotation().Degrees();

    frc::SmartDashboard::PutNumber("targetX", targetX.value());
    frc::SmartDashboard::PutNumber("targetY", targetY.value());
    frc::SmartDashboard::PutNumber("targetRot", targetRot.value());

    // Calculate the path length based on where we are and where we want to go
    double pathLen = m_drive.GetCurrentPose().Translation().Distance(targetPose.Translation()).value();
    m_dbvDistToTag.Put(pathLen);

    m_dbv1Meter.Put(false);
    m_dbv2Meter.Put(false);
    m_dbv3Meter.Put(false);
    m_dbv4Meter.Put(false);

    if (pathLen >= 4.0)
    {
      m_dbv4Meter.Put(true);
    }
    else if (pathLen >= 3.0)
    {
      m_dbv3Meter.Put(true);
    }
    else if (pathLen >= 2.0)
    {
      m_dbv2Meter.Put(true);
    }
    else if (pathLen >= 1.0)
    {
      m_dbv1Meter.Put(true);
    }

#ifdef LED
    if (pathLen > 0.6 && pathLen < 1.00) // This defines the sweet spot for scoring with the on the fly path
    {
      m_led.SetCurrentAction(LEDSubsystem::kTagVisible);
      m_led.SetAnimation(c_colorWhite, LEDSubsystem::kStrobe);
    }
    else if ((m_led.GetCurrentAction() == LEDSubsystem::kTagVisible) && commandRunning == false) //commandRunning is meant to make it so that it doesn't idle when the on the fly path is running
    {
      // m_led.SetCurrentAction(LEDSubsystem::kIdle);
      m_led.SetCurrentAction(LEDSubsystem::kHasCoral);
      m_led.SetAnimation(c_colorPink, LEDSubsystem::kSolid);
    }
#endif
  }
}

bool RobotContainer::GetTagPose(Pose2d& tagPose)
{
  int tagId = m_vision.GetTagId(); // Returns -1 if no tag being imaged
  auto it = c_mapTagPoses.find(tagId);
  if (it != c_mapTagPoses.end())
  {
    tagPose = it->second.GetPose(m_sideSelected);

    return true;
  }

  return false;
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
        constexpr double kDeadband = 0.02;
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
#ifdef USE_SECOND_XBOX_CONTROLLER
  ConfigSecondaryButtonBindings();
#endif
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
  primary.A().OnTrue(frc2::SequentialCommandGroup{
      m_setHighSpeedCmd
    , CoralEjectCommand(*this)
    , ElevatorGoToCommand(*this, L1)  // ElevatorGoToCommand will use algaeL2_3 if too high
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L1)
  }.ToPtr());
  primary.B().OnTrue(CoralPrepCommand(*this).ToPtr());
  primary.X().OnTrue(ClimbDeployCommand(*this).ToPtr());

  primary.Y().OnTrue(InstantCommand{[this] { CommandScheduler::GetInstance().CancelAll(); }, {} }.ToPtr());

  primary.Back().OnTrue(ClimbRetractCommand(*this).ToPtr());
  primary.Start().OnTrue(&m_toggleSlowSpeed);
  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.RightBumper().OnTrue(DeferredCommand(GetFollowPathCommand, {&m_drive} ).ToPtr());

  primary.POVUp().OnTrue(StopAllCommand(*this).ToPtr());
  primary.POVDown().OnTrue(&m_intakeAlign);
  primary.POVLeft().OnTrue(&m_resetOdo);                                // Aligns the gyro with the tag currently being imaged
  primary.POVRight().OnTrue(ResetAngleAtLoadCommand(*this).ToPtr());    // Uses blue/red and Y position to decide angle and sign
}

#ifdef USE_SECOND_XBOX_CONTROLLER
void RobotContainer::ConfigSecondaryButtonBindings()
{
  auto& secondary = m_secondaryController;

  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
  secondary.A().OnTrue(&m_elevL2);
  secondary.B().OnTrue(&m_elevL3);
  secondary.X().OnTrue(frc2::SequentialCommandGroup{
    m_setHighSpeedCmd
    , ElevatorGoToCommand(*this, L1)  // ElevatorGoToCommand will use algaeL2_3 if too high
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L1)
  }.ToPtr());
  secondary.Y().OnTrue(&m_elevL4);

  secondary.LeftBumper().OnTrue(CoralEjectPostCommand(*this).ToPtr());
  secondary.RightBumper().OnTrue(frc2::SequentialCommandGroup{
      CoralPrepCommand(*this)
    , ConditionalCommand (SequentialCommandGroup{
                                ElevatorGoToCommand(*this, L4, c_bUsePresetLevel)
                              , InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} } }, 
                          InstantCommand{[this] {m_coral.RetractManipulator(); }, {&m_coral} }, 
                          [this](){return (m_elevator.GetPresetLevel() == L4 || m_elevator.GetPresetLevel() == L1);})
    , WaitCommand(0.55_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
  }.ToPtr());
      
  // secondary.Back().OnTrue(&m_elevReset);
  secondary.Back().OnTrue(frc2::SequentialCommandGroup{
    m_setHighSpeedCmd
    , ElevatorGoToCommand(*this, L1)  // ElevatorGoToCommand will use algaeL2_3 if too high
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L1)
    , m_elevReset
  }.ToPtr());
  secondary.Start().OnTrue(&m_coralRetract);
  secondary.POVDown().OnTrue(&m_elevRelPosDown);
  secondary.POVUp().OnTrue(&m_elevRelPosUp);
  secondary.LeftTrigger().OnTrue(&m_elevL3_4);
  secondary.RightTrigger().OnTrue(&m_elevL2_3);
  secondary.LeftStick().OnTrue(&m_intakeAlign);
  //secondary.RightStick().OnTrue(&m_intakeParkForClimb);
  secondary.POVRight().OnTrue(&m_coralDeployManip);
  secondary.POVLeft().OnTrue(&m_coralRetractManip);

  secondary.RightTrigger().OnTrue(InstantCommand{[this](){ m_drive.JogRotate(true); }, {&m_drive} }.ToPtr());
  secondary.LeftTrigger().OnTrue(InstantCommand{[this](){ m_drive.JogRotate(false); }, {&m_drive} }.ToPtr());

#ifdef TEST_WHEEL_CONTROL
  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVUp(loop).Rising().IfHigh([this] { m_wheelsLeft.Schedule(); });
  secondary.POVRight(loop).Rising().IfHigh([this] { m_wheelsForward.Schedule(); });
#endif  //  TEST_WHEEL_CONTROL
}
#endif

#ifdef USE_BUTTON_BOX
void RobotContainer::ConfigButtonBoxBindings()
{
  auto& buttonBox = m_buttonController;

  // Physical layout and XBox assignment
  // ┌───────┌───────┬───────┐───────┐
  // │Green1 │White2 │ Blue2 │Green1 │
  // │  X    │  Back │ Start │  DU   │
  // ├───────├───────┼───────┤───────┤
  // │Yellow1│Green2 │ Red2  │ Blue3 │
  // │  Y    │  LS   │  RS   │  DD   │
  // ├───────├───────┼───────┤───────┤
  // │ Blue1 │Black2 │Yellow2│ Red3  │
  // │  RB   │  B    │  A    │  DR   │
  // ├───────┼───────┼───────┤───────┤
  // │Black1 │White1 │ Red1  │Yellow3│        
  // │  LB   │   LT  │  RT   │  DL   │        
  // └───────┴───────┴───────┘───────┘  
  buttonBox.X().OnTrue(&m_setL4);
  buttonBox.Y().OnTrue(frc2::SequentialCommandGroup{
      m_elevL3
    , m_setL3
  }.ToPtr());
  buttonBox.RightBumper().OnTrue(frc2::SequentialCommandGroup{
      m_elevL2
    , m_setL2
  }.ToPtr());
  buttonBox.LeftBumper().OnTrue(frc2::SequentialCommandGroup{
      ElevatorGoToCommand(*this, L1)  // ElevatorGoToCommand will use algaeL2_3 if too high
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, L1)
  }.ToPtr());

  buttonBox.Back().OnTrue(&m_elevRelPosUp);
  buttonBox.LeftStick().OnTrue(&m_elevRelPosDown);
  buttonBox.B().OnTrue(frc2::SequentialCommandGroup{
      m_elevL1
    , CoralIntakeCommand(*this)
    , m_elevL3
  }.ToPtr());
  buttonBox.LeftTrigger().OnTrue(&m_setLeft);

  buttonBox.Start().OnTrue(&m_elevL3_4);
  buttonBox.RightStick().OnTrue(&m_elevL2_3);
  buttonBox.A().OnTrue(frc2::SequentialCommandGroup{      // Score
      m_FollowPathLED
    , DeferredCommand(GetFollowPathCommand, {&m_drive} )
    , CoralPrepCommand(*this)
    , ConditionalCommand (SequentialCommandGroup{
                                //WaitCommand(0.20_s)
                                ElevatorGoToCommand(*this, L4, c_bUsePresetLevel)
                              , InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} } }, 
                          InstantCommand{[this] {m_coral.RetractManipulator(); }, {&m_coral} }, 
                          [this](){return m_elevator.GetPresetLevel() == L4;})
//                          [this](){return (m_elevator.GetPresetLevel() == L4 || m_elevator.GetPresetLevel() == L1);})
    , WaitCommand(0.55_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
    , m_elevL1
    , m_EndLED
  }.ToPtr());
  buttonBox.RightTrigger().OnTrue(&m_setRight);

  buttonBox.POVUp().OnTrue(&m_ClimberDeployRelUp);        // Jog out
  buttonBox.POVDown().OnTrue(&m_ClimberDeployRelDown);    // Jog in
  buttonBox.POVRight().OnTrue(frc2::SequentialCommandGroup{
      m_FollowPathLED
    , CoralPrepCommand(*this)
    , ConditionalCommand (SequentialCommandGroup{
                                ElevatorGoToCommand(*this, L4, c_bUsePresetLevel)
                              , InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} } }, 
                          InstantCommand{[this] {m_coral.RetractManipulator(); }, {&m_coral} }, 
                          [this](){return (m_elevator.GetPresetLevel() == L4 || m_elevator.GetPresetLevel() == L1);})
    , WaitCommand(0.55_s)
    , CoralEjectCommand(*this)
    , WaitCommand(0.25_s)
    , CoralEjectPostCommand(*this)
    , m_elevL1
    , m_EndLED
  }.ToPtr());

#define DRIVE_BACK
#ifdef DRIVE_BACK
  buttonBox.POVLeft().OnTrue(frc2::SequentialCommandGroup{        // Drive back
      InstantCommand{[this](){m_drive.WheelsForward();}, {&m_drive}}
    , WaitCommand(0.125_s)
    , InstantCommand{[this](){m_drive.Stop();}, {&m_drive}}
  }.ToPtr());
#else
  buttonBox.POVLeft().OnTrue(frc2::SequentialCommandGroup{
        m_setHighSpeedCmd
      , DeferredCommand(GetFollowPathCommand, {&m_drive} )
      , PositionPIDCommand(*this, m_targetPose)  
  }.ToPtr());
#endif

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
  //printf("auto chosen %s\n", m_chooser.GetSelected()->GetName().c_str());
  return m_chooser.GetSelected();
}

void RobotContainer::StopAll()
{
  m_intake.Stop();
  m_coral.Stop();
  m_elevator.Stop();
  //m_climb.Stop();
  m_drive.Stop();
#ifdef LED
  m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  m_led.SetCurrentAction(LEDSubsystem::kIdle);
#endif
  // TODO scheduler cancel???
}

void RobotContainer::SetSideSelected(ESideSelected sideSelected)
{
  m_sideSelected = sideSelected;
  frc::SmartDashboard::PutBoolean("RightSelected", (m_sideSelected == RightSide));
  frc::SmartDashboard::PutBoolean("LeftSelected", (m_sideSelected == LeftSide));
}

frc2::CommandPtr RobotContainer::GetFollowPathCommandImpl()
{
  Pose2d& targetPose = m_targetPose;
  if (!GetTagPose(targetPose))
  {
    // TODO LEDs and alert message?
    return frc2::PrintCommand("No tag being imaged right now").ToPtr();
  }

#define USE_POSITION_PID
#ifdef USE_POSITION_PID
  commandRunning = true;
  double pathLen = m_drive.GetCurrentPose().Translation().Distance(targetPose.Translation()).value();
  if (pathLen < 0.01)   // 10cm ~ 4in
  {
      printf("pathLen %.3f running PositionPIDCommand\n", pathLen);
      return PositionPIDCommand(*this, targetPose).ToPtr();
  }
#endif

  auto p = SmartDashboard::GetNumber("PathTransP", DriveConstants::c_HolomonicTranslateP);
#ifdef USE_POSITION_PID_IN_SEQ
  return frc2::SequentialCommandGroup(
    FollowPathCommand(
        GetOnTheFlyPath()
      , [this]() { return m_drive.GetPose(); } // Function to supply current robot pose
      , [this]() { return m_drive.GetChassisSpeeds(); }
      , [this](const frc::ChassisSpeeds& speeds, const DriveFeedforwards &dffs) { m_drive.Drive(speeds, dffs); } // Output function that accepts field relative ChassisSpeeds
      , std::dynamic_pointer_cast<PathFollowingController>(std::make_shared<PPHolonomicDriveController>(PIDConstants(p, 0.0, 0.0), 
                                                                                                        PIDConstants(DriveConstants::c_HolomonicRotateP, 0.0, 0.0)))
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
    , PositionPIDCommand(*this, targetPose)
  ).ToPtr();
#else
  return FollowPathCommand(
        GetOnTheFlyPath()
      , [this]() { return m_drive.GetPose(); } // Function to supply current robot pose
      , [this]() { return m_drive.GetChassisSpeeds(); }
      , [this](const frc::ChassisSpeeds& speeds, const DriveFeedforwards &dffs) { m_drive.Drive(speeds, dffs); } // Output function that accepts field relative ChassisSpeeds
      , std::dynamic_pointer_cast<PathFollowingController>(std::make_shared<PPHolonomicDriveController>(PIDConstants(p, 0.0, 0.0), 
                                                                                                        PIDConstants(DriveConstants::c_HolomonicRotateP, 0.0, 0.0)))
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
      commandRunning = false;
#endif
}

std::shared_ptr<PathPlannerPath> RobotContainer::GetOnTheFlyPath()
{
  std::shared_ptr<PathPlannerPath> path;

  m_drive.SetSlowSpeed(true);

  units::length::meter_t targetX;
  units::length::meter_t targetY;
  units::angle::degree_t targetRot;

  Pose2d targetPose;
  if (!GetTagPose(targetPose)) // GetTagPose returns false if no tag being imaged
  {
      return path;
  }

  targetX = targetPose.X();
  targetY = targetPose.Y();
  targetRot = targetPose.Rotation().Degrees();

  frc::SmartDashboard::PutNumber("targetX", targetX.value());
  frc::SmartDashboard::PutNumber("targetY", targetY.value());
  frc::SmartDashboard::PutNumber("targetRot", targetRot.value());

  // Calculate the heading (tanAngle) based on where we are and where we want to go
  auto currentPose = m_drive.GetCurrentPose();
  auto currentX = currentPose.X();
  auto currentY = currentPose.Y();
  //double xDelta = targetX.value() - currentX.value();
  //double yDelta = targetY.value() - currentY.value();
  
  units::velocity::meters_per_second_t xSpeed = m_drive.GetChassisSpeeds().vx;
  units::velocity::meters_per_second_t ySpeed = m_drive.GetChassisSpeeds().vy;
  double xSpeed2 = static_cast<double>(xSpeed);
  double ySpeed2 = static_cast<double>(ySpeed);
  auto startRot = frc::Rotation2d(xSpeed2, ySpeed2);
  //auto tanAngle = units::angle::degree_t{atan(yDelta / xDelta) * 180.0 / std::numbers::pi};
  //printf("tanAngle %.3f\n", tanAngle);
//  std::vector<frc::Pose2d> poses {  frc::Pose2d { currentX, currentY, frc::Rotation2d(xSpeed2, ySpeed2) } // USED TO BE tanAngle
  std::vector<frc::Pose2d> poses {  frc::Pose2d { currentX, currentY, startRot } // USED TO BE tanAngle
                                  , frc::Pose2d { targetX, targetY, targetRot }
  };

  path = std::make_shared<PathPlannerPath>(
      PathPlannerPath::waypointsFromPoses(poses),
      m_pathConstraints,
      // If the robot is already in motion this should prevent jerky behavior (see Spartronics 4915 https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit)
      IdealStartingState(m_drive.GetSpeed(), currentPose.Rotation()),
      GoalEndState(0.0_mps, targetPose.Rotation())
  );

  // Prevent the path from being flipped if the coordinates are already correct
  path->preventFlipping = true;

//#define PRINT_PATH
#ifdef PRINT_PATH
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
#endif // def PRINT_PATH

  // Do not need to reset odemettry for one the fly path?
  // for testing only m_drive.ResetOdometry(Pose2d{currentX, currentY, targetRot});

  return path;
}

void RobotContainer::ConfigureRobotLEDs()
{
#ifdef LED
  GetLED().Periodic();
#endif
}