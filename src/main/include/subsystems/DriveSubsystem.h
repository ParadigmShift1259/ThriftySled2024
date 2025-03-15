// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#ifndef __DRIVESUBSYSTEM_H__
#define __DRIVESUBSYSTEM_H__

#include <wpi/DataLog.h>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Preferences.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <networktables/StructTopic.h>

#include <pathplanner/lib/path/PathPlannerPath.h>

#include "IDriveSubsystem.h"
#include "ConstantsCANIDs.h"
#include "subsystems/SwerveModule.h"
#include "PigeonGyro.h"
#include "DashBoardValue.h"

//#define SIMULATION
#ifndef SIMULATION
#include "LimelightHelpers.h"
#endif

// TODO These speeds should be divided by the drive gear ratio 
static constexpr units::meters_per_second_t kMaxSpeed = 16.8_fps;  // Thrifty 18P13 Gear Ratio Kraken Max Speed
static constexpr units::meters_per_second_t kSlowSpeed = 1.0_fps;

static constexpr units::radians_per_second_t kMaxAngularSpeed{2.5 * std::numbers::pi};  // 2 1/2 radians per second
static constexpr units::radians_per_second_t kLowAngularSpeed{ std::numbers::pi/3.0};  // 1/2 radian per second

static constexpr units::radians_per_second_t kRotationDriveMaxSpeed = 7.5_rad_per_s;
static constexpr units::radians_per_second_t kRotationDriveDirectionLimit = 7.0_rad_per_s;

static constexpr units::radians_per_second_t kAimingRotationDriveMaxSpeed = 7.5_rad_per_s;
static constexpr units::radians_per_second_t kAimingRotationDriveDirectionLimit = 7.0_rad_per_s;

using namespace pathplanner;

/**
 * Represents a swerve drive style DriveSubsystem.
 */
class DriveSubsystem : public frc2::SubsystemBase, public IDriveSubsystem
{
public:
  DriveSubsystem();

  void Drive(units::meters_per_second_t xSpeed
           , units::meters_per_second_t ySpeed
           , units::radians_per_second_t rot
           , bool fieldRelative) override;

  void RotationDrive(units::meters_per_second_t xSpeed
                   , units::meters_per_second_t ySpeed
                   , double xRot
                   , double yRot
                   , bool fieldRelative) override; 

  void RotationDrive(units::meters_per_second_t xSpeed
                   , units::meters_per_second_t ySpeed
                   , units::radian_t rot
                   , bool fieldRelative);

  void Drive(const frc::ChassisSpeeds& speeds, const DriveFeedforwards& dffs);

  void UpdateOdometry() override;
  void ResetOdometry(frc::Pose2d pose) override;
  void Periodic() override;
  units::degree_t GetPitch() override { return m_gyro.GetPitch(); }
  frc::Pose2d GetPose() override;
  frc::ChassisSpeeds GetChassisSpeeds() override;
  units::velocity::meters_per_second_t GetSpeed();
  void SetModuleStates(SwerveModuleStates desiredStates) override;

  frc::Pose2d GetCurrentPose() { return m_poseEstimator.GetEstimatedPosition(); }
  units::length::meter_t GetX() override { return m_poseEstimator.GetEstimatedPosition().X(); }
  units::length::meter_t GetY() override { return m_poseEstimator.GetEstimatedPosition().Y(); }

  void ResyncAbsRelEnc() override;
  void SetOverrideXboxInput(bool bOverride) override { m_bOverrideXboxInput = bOverride; }
  void JogRotate(bool bClockwise);
  void WheelsForward() override;
  void WheelsLeft() override;
  void WheelsBackward() override;
  void WheelsRight() override;

  void Stop() {m_bOverrideXboxInput = false;}

  TalonFX& GetTalon(int module);

  units::angle::degree_t GetGyroAzimuthDeg() { return m_gyro.GetRotation2d().Degrees(); }

  RobotConfig GetRobotCfg() { return m_robotConfig; }

  void SetSlowSpeed(bool slow) 
  {
    m_currentMaxSpeed = (slow ? kSlowSpeed : kMaxSpeed);
    m_currentMaxAngularSpeed = (slow ? kLowAngularSpeed : kMaxAngularSpeed);

    m_frontLeft.SetMaxSpeed(m_currentMaxSpeed);
    m_frontRight.SetMaxSpeed(m_currentMaxSpeed);
    m_rearLeft.SetMaxSpeed(m_currentMaxSpeed);
    m_rearRight.SetMaxSpeed(m_currentMaxSpeed);
  }

  void ToggleSlowSpeed() override
  { 
    SetSlowSpeed(m_currentMaxSpeed == kMaxSpeed);
  }

  units::meters_per_second_t m_currentMaxSpeed = kSlowSpeed;//kMaxSpeed;
  units::radians_per_second_t m_currentMaxAngularSpeed = kMaxAngularSpeed;

private:
  void SetAllDesiredState(frc::SwerveModuleState& sms);
  wpi::array<frc::SwerveModuleState, 4> GetModuleStates() { return {m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(), m_rearRight.GetState()}; }

  static constexpr auto kTrackWidth = 24_in;
  static constexpr auto kWheelBase = 24_in;
  const frc::Translation2d m_frontLeftLocation{  kWheelBase / 2,  kTrackWidth / 2 };
  const frc::Translation2d m_frontRightLocation{ kWheelBase / 2, -kTrackWidth / 2 };
  const frc::Translation2d m_rearLeftLocation{  -kWheelBase / 2,  kTrackWidth / 2 };
  const frc::Translation2d m_rearRightLocation{ -kWheelBase / 2, -kTrackWidth / 2 };

  const double kFLoffset = frc::Preferences::GetDouble("Offset1", 0.0);
  const double kFRoffset = frc::Preferences::GetDouble("Offset2", 0.0);
  const double kBLoffset = frc::Preferences::GetDouble("Offset4", 0.0);
  const double kBRoffset = frc::Preferences::GetDouble("Offset3", 0.0);

  SwerveModule m_frontLeft  { kFrontLeftDriveCANID, kFrontLeftTurningCANID, kFLoffset, true };     // 1
  SwerveModule m_frontRight { kFrontRightDriveCANID, kFrontRightTurningCANID, kFRoffset, false };  // 2
  SwerveModule m_rearLeft   { kRearLeftDriveCANID, kRearLeftTurningCANID, kBLoffset, true };       // 4
  SwerveModule m_rearRight  { kRearRightDriveCANID, kRearRightTurningCANID, kBRoffset, false };    // 3

  PigeonGyro m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics
  {
      m_frontLeftLocation, m_frontRightLocation, 
      m_rearLeftLocation, m_rearRightLocation
  };


  // For PathPlanner on the fly paths
  ModuleConfig m_moduleCfg;
  RobotConfig m_robotConfig;

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator
  {
      m_kinematics
    , m_gyro.GetRotation2d()
    , { m_frontLeft.GetPosition() , m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition() }
    , frc::Pose2d{}
    , { 0.1, 0.1, 0.1 }       // std deviations of the pose estimate; increase these numbers to trust your state estimate less.
    , { 0.7, 0.7, 9999999.0 } // std deviations of the vision pose measurement; increase these numbers to trust the vision pose measurement less.
  };

  frc::PIDController m_rotationPIDController{ 1.0, 0.0, 0.025 };

  bool m_bOverrideXboxInput = false;

  DashBoardValue<double> m_dbvPitch{"Gyro", "Pitch", m_gyro.GetPitch().value()};
  DashBoardValue<double> m_dbvRoll{"Gyro", "Roll", m_gyro.GetRoll().value()};

  // Logging Member Variables
  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
  // wpi::log::DoubleLogEntry m_logRobotSpeed;
  // wpi::log::DoubleLogEntry m_logRobotAccel;
  wpi::log::DoubleLogEntry m_logGyroPitch;

  wpi::log::DoubleLogEntry m_logDriveInputX;
  wpi::log::DoubleLogEntry m_logDriveInputY;
  wpi::log::DoubleLogEntry m_logDriveInputRot;
};

#endif  //ndef __DRIVESUBSYSTEM_H__
