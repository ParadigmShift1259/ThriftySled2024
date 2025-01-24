// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/GoToPositionCommand.h"
#include "commands/ElevatorGoToCommand.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DriverStation.h>

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/SequentialCommandGroup.h>

  // Configure the button bindings
RobotContainer::RobotContainer() 
  : m_drive()
#ifdef USE_ORCESTRA
  , m_orchestra("output.chrp")
#endif
{
  //---------------------------------------------------------
  //printf("************************Calling SilenceJoystickConnectionWarning - Wisco2024 Day 1 only REMOVE!!!!!\n");
  //DriverStation::SilenceJoystickConnectionWarning(true);
  //---------------------------------------------------------
  SetDefaultCommands();
  ConfigureBindings();

  frc::SmartDashboard::PutNumber("elevHeight", 0.0);
  frc::SmartDashboard::PutNumber("elevDelay", 0.015);
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return autos::ExampleAuto(&m_subsystem);
// }

void RobotContainer::Periodic()
{
  m_drive.Periodic();
  //static int count = 0;
  //if (count++ % 25 == 0)
  //{
    frc::SmartDashboard::PutBoolean("FieldRelative", m_fieldRelative);
  //}
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
// define USE_XBOX in RobotContainer.h
#ifdef USE_XBOX      
        const auto xInput = direction* ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
        const auto yInput = direction * ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
        const auto rotInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);      
        // const auto rotXInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);
        // const auto rotYInput = ApplyDeadband(m_primaryController.GetRightY(), kDeadband);
        // const double rotX = m_rotLimiter.Calculate(rotXInput);
        // const double rotY = m_rotLimiter.Calculate(rotYInput);
#else
        constexpr double kRotDeadband = 0.42;
        const auto xInput = direction* ApplyDeadband(m_primaryController.GetHID().GetY(), kDeadband);
        const auto yInput = direction * ApplyDeadband(m_primaryController.GetHID().GetX(), kDeadband);
        const auto rotInput = ApplyDeadband(m_secondaryController.GetHID().GetX(), kRotDeadband);
        // rotInput *= fabs(rotInput);      
#endif
        const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * m_drive.m_currentMaxSpeed;
        auto ySpeed = m_yspeedLimiter.Calculate(yInput) * m_drive.m_currentMaxSpeed;
        auto rot = m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;      

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
  // ConfigButtonBoxBindings();
#endif
}

void RobotContainer::
ConfigPrimaryButtonBindings()
{
  auto& primary = m_primaryController;
 
  // Primary
  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
  /*primary.A().WhileTrue(GoToPositionCommand(*this, false).ToPtr());
  primary.B().WhileTrue(frc2::SequentialCommandGroup{
    GoToAzimuthCommand(*this)
    , m_posPipeline
  }.ToPtr());*/

  //primary.X().OnTrue(&m_trapRPM);
#ifdef USE_XBOX
  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.RightBumper().WhileTrue(GoToPositionCommand(*this, eMiddle).ToPtr());
  primary.B().WhileTrue(GoToPositionCommand(*this, eRight).ToPtr());
  primary.X().WhileTrue(GoToPositionCommand(*this, eLeft).ToPtr());
  primary.POVUp().OnTrue(GoToPositionCommand(*this, eJogForward).ToPtr());
  primary.POVDown().OnTrue(GoToPositionCommand(*this, eJogBackward).ToPtr());
  primary.POVRight().OnTrue(GoToPositionCommand(*this, eJogRight).ToPtr());
  primary.POVLeft().OnTrue(GoToPositionCommand(*this, eJogLeft).ToPtr());
#else
  primary.Button(7).OnTrue(&m_toggleFieldRelative);
#endif
  primary.Back().OnTrue(&m_resetOdo);
}

void RobotContainer::ConfigSecondaryButtonBindings()
{
  auto& secondary = m_secondaryController;

  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
//  secondary.A().OnTrue(frc2::SequentialCommandGroup{
  secondary.Y().OnTrue(&m_elevL4);
  secondary.B().OnTrue(&m_elevL3);
  secondary.A().OnTrue(&m_elevL2);
  secondary.X().OnTrue(frc2::SequentialCommandGroup{
    ElevatorGoToCommand(*this, 2.0)
    , WaitCommand(0.4_s)
    , ElevatorGoToCommand(*this, 0.0)
  }.ToPtr());
      
  secondary.Back().OnTrue(&m_elevReset);
  secondary.POVDown().OnTrue(&m_elevRelPosDown);
  secondary.POVUp().OnTrue(&m_elevRelPosUp);

#ifdef TEST_WHEEL_CONTROL
  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVUp(loop).Rising().IfHigh([this] { m_wheelsLeft.Schedule(); });
  secondary.POVRight(loop).Rising().IfHigh([this] { m_wheelsForward.Schedule(); });
#endif  //  TEST_WHEEL_CONTROL
}

void RobotContainer::StopAll(){
  m_elevator.Stop();
}