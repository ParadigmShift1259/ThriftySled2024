// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

DriveSubsystem::DriveSubsystem()
  : m_gyro(kDrivePigeonCANID)
{
  m_gyro.Reset();

  wpi::log::DataLog& log = frc::DataLogManager::GetLog();
  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseTheta");   
  m_logGyroPitch = wpi::log::DoubleLogEntry(log, "/drivegyro/pitch");

  m_logDriveInputX = wpi::log::DoubleLogEntry(log, "/input/X");
  m_logDriveInputY = wpi::log::DoubleLogEntry(log, "/input/Y");
  m_logDriveInputRot = wpi::log::DoubleLogEntry(log, "/input/Rot");

// This block of code is to remove last season's constants stored in preferences
//#define REMOVE_OLD_PREFERENCES
#ifdef REMOVE_OLD_PREFERENCES
  auto keys = frc::Preferences::GetKeys();
  for (const auto& key : keys)
  {
    if (!key.empty() && key[0] == 'k')
    {
      frc::Preferences::Remove(key);
    }
  }
#endif  // REMOVE_OLD_PREFERENCES
  frc::Preferences::SetString("BuildDate", __DATE__);
  frc::Preferences::SetString("BuildTime", __TIME__);

  frc::Preferences::InitString("Name", "ThingX");
  frc::Preferences::InitDouble("Offset1", 0.0);
  frc::Preferences::InitDouble("Offset2", 0.0);
  frc::Preferences::InitDouble("Offset3", 0.0);
  frc::Preferences::InitDouble("Offset4", 0.0);
  
  frc::SmartDashboard::PutNumber("yawsign", 1.0);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative)
{
  //printf("DriveSubsystem::Drive xspd %.3f yspd %.3f rot %.3f fldrel %d\n", xSpeed.value(), ySpeed.value(), rot.value(), fieldRelative);
  m_logDriveInputX.Append(xSpeed.to<double>());
  m_logDriveInputY.Append(ySpeed.to<double>());
  m_logDriveInputRot.Append(rot.to<double>());

  m_frontLeft.SetMaxSpeed(m_currentMaxSpeed);
  m_frontRight.SetMaxSpeed(m_currentMaxSpeed);
  m_rearLeft.SetMaxSpeed(m_currentMaxSpeed);
  m_rearRight.SetMaxSpeed(m_currentMaxSpeed);

  // frc::SmartDashboard::PutNumber("Input x speed", xSpeed.to<double>());
  // frc::SmartDashboard::PutNumber("Input y speed", ySpeed.to<double>());
  // frc::SmartDashboard::PutNumber("Input rot", rot.to<double>());

  if (m_bOverrideXboxInput == false)
  {
    auto states = m_kinematics.ToSwerveModuleStates(fieldRelative 
       ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
       : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

    // Renormalizes the wheel speeds if any individual speed is above the specified maximum
    m_kinematics.DesaturateWheelSpeeds(&states, m_currentMaxSpeed);//kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_rearLeft.SetDesiredState(bl);
    m_rearRight.SetDesiredState(br);
  }
}

//Field Relative, Changes rotation to match the field
void DriveSubsystem::RotationDrive(units::meters_per_second_t xSpeed
                                 , units::meters_per_second_t ySpeed
                                 , units::radian_t rot
                                 , bool fieldRelative) 
{  
  auto error = rot - m_gyro.GetRotation2d().Radians();//m_gyro->GetHeadingAsRot2d().Radians().to<double>();
  frc::SmartDashboard::PutNumber("turnError", error.value());
  if (error.to<double>() > std::numbers::pi)
  {
    error -= units::radian_t(2 * std::numbers::pi);
  }
  else if (error.to<double>() < -1 * std::numbers::pi)
  {
    error += units::radian_t(2 * std::numbers::pi);
  }

  auto max = kRotationDriveMaxSpeed;
  auto maxTurn = kRotationDriveDirectionLimit;
  
  bool isAiming = frc::SmartDashboard::GetBoolean("IsAiming", false);
  
  if (isAiming)
  {
    max = kAimingRotationDriveMaxSpeed;
    // maxTurn = kAimingRotationDriveDirectionLimit;
  }

  #ifdef TUNE_ROTATION_DRIVE
  double P = SmartDashboard::GetNumber("T_D_RP", 0);
  double I = SmartDashboard::GetNumber("T_D_RI", 0);
  double D = SmartDashboard::GetNumber("T_D_RD", 0);
  double m = SmartDashboard::GetNumber("T_D_RMax", 0);
  double mTurn = SmartDashboard::GetNumber("T_D_RTMax", 0);

  m_rotationPIDController.SetP(P);
  m_rotationPIDController.SetI(I);
  m_rotationPIDController.SetD(D);
  max = m;
  maxTurn = mTurn;
  #endif

  units::radians_per_second_t desiredTurnRate(m_rotationPIDController.Calculate(0, error.to<double>()));

  units::radians_per_second_t currentTurnRate = m_gyro.GetTurnRate();

  // Prevent sharp turning if already fast going in the opposite direction
  if ((units::math::abs(currentTurnRate) >= maxTurn) && (std::signbit(desiredTurnRate.to<double>()) != std::signbit(currentTurnRate.to<double>())))
      desiredTurnRate *= -1.0;

  // Power limiting
  if (units::math::abs(desiredTurnRate) > max)
      desiredTurnRate = std::signbit(desiredTurnRate.to<double>()) ? max * -1.0 : max;

  Drive(xSpeed, ySpeed, desiredTurnRate, fieldRelative);
}

void DriveSubsystem::RotationDrive(units::meters_per_second_t xSpeed
                                 , units::meters_per_second_t ySpeed
                                 , double xRot
                                 , double yRot
                                 , bool fieldRelative) 
{
  if (xRot != 0 || yRot != 0)
    {
      RotationDrive(xSpeed, ySpeed, units::radian_t(atan2f(yRot, xRot)), fieldRelative);
    }
    else
    {
      Drive(xSpeed, ySpeed, units::radians_per_second_t(0), fieldRelative);
    }  
    
}

void DriveSubsystem::Periodic()
{
  UpdateOdometry();
  m_frontLeft.Periodic();
  m_frontRight.Periodic();
  m_rearLeft.Periodic();
  m_rearRight.Periodic();

  // Log Odometry Values
  frc::Pose2d pose = m_poseEstimator.GetEstimatedPosition();
  //frc::Pose2d pose = m_odometry.GetPose();

  m_logRobotPoseX.Append(pose.X().to<double>());
  m_logRobotPoseY.Append(pose.Y().to<double>());
  m_logRobotPoseTheta.Append(pose.Rotation().Degrees().to<double>());
  m_logGyroPitch.Append(m_gyro.GetPitch()); 
  frc::SmartDashboard::PutNumber("currPoseRadians", GetGyroAzimuth().value());

  frc::SmartDashboard::PutNumber("azimuthDeg", m_gyro.GetRotation2d().Degrees().value());

  static int count = 0;
  if (count++ % 25 == 0)
  {
    frc::SmartDashboard::PutBoolean("SlowSpeed", m_currentMaxSpeed == kLowSpeed);
  }

  // Update limelight for megatag2
  //LimelightHelpers::SetRobotOrientation("limelight-reef", m_gyro.GetYaw().value(), 0.0, 0.0, 0.0, 0.0, 0.0);

  bool doUpdate = true;
  LimelightHelpers::SetRobotOrientation("limelight-reef", m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
  LimelightHelpers::PoseEstimate mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-reef");
  if(fabs(m_gyro.GetTurnRate().value()) > 720.0) {
    doUpdate = false;
  }
  if(doUpdate){
    //const wpi::array<double, 3> stdDevs {.6, .6, 9999999};
    //m_poseEstimator.SetVisionMeasurementStdDevs(stdDevs);
    m_poseEstimator.AddVisionMeasurement(mt2.pose, mt2.timestampSeconds);
  }
}

frc::Pose2d DriveSubsystem::GetPose()
{
  return m_poseEstimator.GetEstimatedPosition();
//  return m_odometry.GetPose();

}

frc::ChassisSpeeds DriveSubsystem::GetChassisSpeeds()
{
  //printf("DriveSubsystem::GetChassisSpeeds fl %.3f fr %.3f rl %.3f rr %.3f\n"
  // , m_frontLeft.GetState().speed.value()
  // , m_frontRight.GetState().speed.value()
  // , m_rearLeft.GetState().speed.value()
  // , m_rearRight.GetState().speed.value());
  return m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(), m_rearRight.GetState()});
}

void DriveSubsystem::ResyncAbsRelEnc()
{
  m_frontLeft.ResyncAbsRelEnc();
  m_frontRight.ResyncAbsRelEnc();
  m_rearLeft.ResyncAbsRelEnc();
  m_rearRight.ResyncAbsRelEnc();
}

void DriveSubsystem::UpdateOdometry()
{
   m_poseEstimator.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_rearLeft.GetPosition(),  m_rearRight.GetPosition()});

  // m_odometry.Update(m_gyro.GetRotation2d(),
  //                  {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
  //                   m_rearLeft.GetPosition(),  m_rearRight.GetPosition()});

  m_publisher.Set(m_poseEstimator.GetEstimatedPosition());
  //m_publisher.Set(m_odometry.GetPose());
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  // frc::SmartDashboard::PutNumber("ResetX", pose.X().to<double>());
  // frc::SmartDashboard::PutNumber("Resety", pose.Y().to<double>());
  // frc::SmartDashboard::PutNumber("ResetRot", pose.Rotation().Degrees().to<double>());
  //printf ("resetx %.3f resety %.3f resetrot %.3f\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());

  SwerveModulePositions modulePositions = {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                           m_rearLeft.GetPosition(), m_rearRight.GetPosition()};
  printf("m_gyro.GetRotation2d().Degrees %.3f pose.Rotation().Degrees %.3f\n", m_gyro.GetRotation2d().Degrees().value(), pose.Rotation().Degrees().value());
  m_gyro.Set(pose.Rotation().Degrees());
  m_poseEstimator.ResetPosition(pose.Rotation(), modulePositions, pose);
  //m_odometry.ResetPosition(pose.Rotation(), modulePositions, pose);
}

void DriveSubsystem::SetHeading(units::degree_t heading)
{
  m_gyro.Set(heading);
}

void DriveSubsystem::WheelsForward()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{0.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::WheelsLeft()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{90.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::WheelsBackward()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{180.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::WheelsRight()
{
  frc::SwerveModuleState sms;
  sms.angle = frc::Rotation2d{-90.0_deg};
  sms.speed = 0.0_mps;
  SetAllDesiredState(sms);
}

void DriveSubsystem::SetAllDesiredState(frc::SwerveModuleState& sms)
{
  m_frontLeft.SetDesiredState(sms);
  m_frontRight.SetDesiredState(sms);
  m_rearLeft.SetDesiredState(sms);
  m_rearRight.SetDesiredState(sms);
}

void DriveSubsystem::SetModuleStates(SwerveModuleStates desiredStates)
{
  m_kinematics.DesaturateWheelSpeeds(&desiredStates, m_currentMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearRight.SetDesiredState(desiredStates[3]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
}

TalonFX& DriveSubsystem::GetTalon(int module)
{
  switch (module) {
    case 0:
      return m_frontLeft.GetTalon(); 
      break;
    case 1:
      return m_frontRight.GetTalon();
      break;
    case 2:
      return m_rearRight.GetTalon();
      break;
    default:
      return m_rearLeft.GetTalon();
      break;
  }
}