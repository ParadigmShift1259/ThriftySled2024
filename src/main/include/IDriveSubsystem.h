#pragma once

#include <numbers>

#include <wpi/array.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

/// Readable alias for array of swerve modules
using SwerveModuleStates = wpi::array<frc::SwerveModuleState, 4>;
using SwerveModulePositions = wpi::array<frc::SwerveModulePosition, 4>;

/**
 * Represents a swerve drive style DriveSubsystem.
 */
class IDriveSubsystem
{
public:
  virtual void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot,
             bool fieldRelative) = 0;
  
  virtual void RotationDrive(units::meters_per_second_t xSpeed
                           , units::meters_per_second_t ySpeed
                           , double xRot
                           , double yRot
                           , bool fieldRelative) = 0;
  
  virtual void UpdateOdometry() = 0;
  virtual void ResetOdometry(frc::Pose2d pose) = 0;
  //virtual void SetHeading(units::degree_t heading) = 0;
  virtual units::degree_t GetPitch() = 0;
  virtual frc::Pose2d GetPose() = 0;
  virtual frc::ChassisSpeeds GetChassisSpeeds() = 0;
  virtual void SetModuleStates(SwerveModuleStates desiredStates) = 0;

  virtual units::length::meter_t  GetX() = 0;
  virtual units::length::meter_t  GetY() = 0;

  virtual void ResyncAbsRelEnc() = 0;
  virtual void SetOverrideXboxInput(bool bOverride) = 0;
  virtual void WheelsForward() = 0;
  virtual void WheelsLeft() = 0;
  virtual void WheelsBackward() = 0;
  virtual void WheelsRight() = 0;

  virtual void ToggleSlowSpeed() = 0;
};
