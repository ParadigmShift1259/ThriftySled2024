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
#include "LimelightHelpers.h"

static constexpr units::meters_per_second_t kMaxSpeed = 18.9_fps;  // L3 Gear Ratio Falcon Max Speed
static constexpr units::meters_per_second_t kLowSpeed = 4.0_fps;  // L3 Gear Ratio Falcon Max Speed
static constexpr units::radians_per_second_t kMaxAngularSpeed{2.5 * std::numbers::pi};  // 1/2 rotation per second
static constexpr units::radians_per_second_squared_t kMaxAngularAcceleration{10.0 * std::numbers::pi};  // 4 rotations per second squared
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
  void SetHeading(units::degree_t heading) override;
  void Periodic() override;
  double GetPitch() override { return m_gyro.GetPitch(); }
  frc::Pose2d GetPose() override;
  frc::ChassisSpeeds GetChassisSpeeds() override;
  void SetModuleStates(SwerveModuleStates desiredStates) override;

  double GetX() override { return m_poseEstimator.GetEstimatedPosition().X().value(); }
  double GetY() override { return m_poseEstimator.GetEstimatedPosition().Y().value(); }

  void ResyncAbsRelEnc() override;
  void SetOverrideXboxInput(bool bOverride) override { m_bOverrideXboxInput = bOverride; }
  void WheelsForward() override;
  void WheelsLeft() override;
  void WheelsBackward() override;
  void WheelsRight() override;

  TalonFX& GetTalon(int module);

  units::angle::radian_t GetGyroAzimuth() { return m_gyro.GetRotation2d().Radians(); }
  units::angle::degree_t GetGyroAzimuthDeg() { return m_gyro.GetRotation2d().Degrees(); }

  RobotConfig GetRobotCfg() { return m_robotConfig; }

  void ToggleSlowSpeed() override
  { 
    m_currentMaxSpeed = (m_currentMaxSpeed == kMaxSpeed ? kLowSpeed : kMaxSpeed);

    m_frontLeft.SetMaxSpeed(m_currentMaxSpeed);
    m_frontRight.SetMaxSpeed(m_currentMaxSpeed);
    m_rearLeft.SetMaxSpeed(m_currentMaxSpeed);
    m_rearRight.SetMaxSpeed(m_currentMaxSpeed);
  }

  units::meters_per_second_t m_currentMaxSpeed = kMaxSpeed;

// Safer sppeds for lab testing
  // static constexpr units::meters_per_second_t kMaxSpeed = 1.0_mps;
  // static constexpr units::radians_per_second_t kMaxAngularSpeed{0.25 * std::numbers::pi};

private:
  void SetAllDesiredState(frc::SwerveModuleState& sms);

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

  // frc::SwerveDriveOdometry<4> m_odometry{
  //     m_kinematics,
  //     m_gyro.GetRotation2d(),
  //     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
  //      m_rearLeft.GetPosition(), m_rearRight.GetPosition()}};

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator
  {
      m_kinematics
    , m_gyro.GetRotation2d()
    , { m_frontLeft.GetPosition() , m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition() }
    , frc::Pose2d{}
    , { 0.1, 0.1, 0.1 }
    , { 0.7, 0.7, 9999999.0 }
  };

  frc::PIDController m_rotationPIDController{ 1.0, 0.0, 0.025 };

  bool m_bOverrideXboxInput = false;

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

  nt::StructPublisher<frc::Pose2d> m_publisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("OdoPose").Publish();
};

#endif  //ndef __DRIVESUBSYSTEM_H__
