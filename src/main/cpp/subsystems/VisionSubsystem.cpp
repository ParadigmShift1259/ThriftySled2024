#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "subsystems/VisionSubsystem.h"

constexpr double c_limelightLoadMountAngle = 27.0;
constexpr double c_limelightReefMountAngle = 30.0;
constexpr units::meter_t c_targetHeight = 55.875_in;
constexpr double c_limelightLoadPositionPipeline = 0.0;
constexpr double c_limelightLoadAnglePipeline = 1.0;

VisionSubsystem::VisionSubsystem()
{
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance)
  {
    m_bIsBlue = (alliance.value() == frc::DriverStation::Alliance::kBlue);
  }


  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logRobotAlliPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseX");
  m_logRobotAlliPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseY");
  m_logRobotAlliPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseTheta");
  m_logLL_Latency = wpi::log::DoubleLogEntry(log, "/vision/LL_Latency");
  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotPoseTheta");
  m_logtxLoad = wpi::log::DoubleLogEntry(log, "/vision/txLoad");
  m_logtyLoad = wpi::log::DoubleLogEntry(log, "/vision/tyShotter");
  m_logtxReef = wpi::log::DoubleLogEntry(log, "/vision/txReef");
  m_logtyReef = wpi::log::DoubleLogEntry(log, "/vision/tyReef");
  m_logtidReef = wpi::log::IntegerLogEntry(log, "/vision/tidReef");

  frc::SmartDashboard::PutBoolean("AllowedLoad", m_isAllowedLoad);
  frc::SmartDashboard::PutBoolean("AllowedReef", m_isAllowedReef);
}

void VisionSubsystem::Periodic()
{
//  PeriodicLoad();
  PeriodicReef();
  m_isAllowedLoad = frc::SmartDashboard::GetBoolean("AllowedLoad", m_isAllowedLoad);
  m_isAllowedReef = true;//frc::SmartDashboard::GetBoolean("AllowedReef", m_isAllowedReef);
}

void VisionSubsystem::PeriodicLoad()
{
  m_isValidLoad = m_netTableLoad->GetNumber("tv", 0) == 1.0;
  if (m_isValidLoad && m_isAllowedLoad)
  {
      m_netBufferField = m_netTableLoad->GetNumberArray("botpose", m_zero_vector);
      m_logRobotPoseX.Append(m_netBufferField[eX]);
      m_logRobotPoseY.Append(m_netBufferField[eY]);
      m_logRobotPoseTheta.Append(m_netBufferField[eYaw]);

      m_netBufferAlli = m_netTableLoad->GetNumberArray("botpose_wpiblue", m_zero_vector);
      m_logRobotAlliPoseX.Append(m_netBufferAlli[eX]);
      m_logRobotAlliPoseY.Append(m_netBufferAlli[eY]);
      m_logRobotAlliPoseTheta.Append(m_netBufferAlli[eYaw]);
      m_logLL_Latency.Append(m_netBufferAlli[eLatency]);

      m_tyLoad = m_netTableLoad->GetNumber("ty", 0.0);
      m_txLoad = m_netTableLoad->GetNumber("tx", 0.0);
      m_logtxLoad.Append(m_txLoad);
      m_logtyLoad.Append(m_tyLoad);



      
      // floorDistance = height from camera to apriltag / tangent + limelight offset from robot
      // m_floorDistance = (45.875 / tan(targetAngle)) + 11.0;
      frc::SmartDashboard::PutNumber("VisionFloorDist", m_floorDistance);
      m_yawError = m_txLoad;
      frc::SmartDashboard::PutNumber("VisionYawError", m_yawError);
  }
  else
  {
    m_yawError = 0.0;
  }
}

void VisionSubsystem::PeriodicReef()
{
  m_isValidReef = m_netTableReef->GetNumber("tv", 0) == 1.0;
  frc::SmartDashboard::PutNumber("tv Reef", m_isValidReef);
  if (m_isValidReef && m_isAllowedReef)
  {
      m_netBufferField = m_netTableReef->GetNumberArray("botpose", m_zero_vector);
      m_logRobotPoseX.Append(m_netBufferField[eX]);
      m_logRobotPoseY.Append(m_netBufferField[eY]);
      m_logRobotPoseTheta.Append(m_netBufferField[eYaw]);

      m_netBufferAlli = m_netTableReef->GetNumberArray("botpose_orb_wpiblue", m_zero_vector);
      m_logRobotAlliPoseX.Append(m_netBufferAlli[eX]);
      m_logRobotAlliPoseY.Append(m_netBufferAlli[eY]);
      m_logRobotAlliPoseTheta.Append(m_netBufferAlli[eYaw]);
      m_logLL_Latency.Append(m_netBufferAlli[eLatency]);

      frc::SmartDashboard::PutNumber("xReef", m_netBufferAlli[eX]);
      frc::SmartDashboard::PutNumber("yReef", m_netBufferAlli[eY]);
      frc::SmartDashboard::PutNumber("yawReef", m_netBufferAlli[eYaw]);

      m_tyReef = m_netTableReef->GetNumber("ty", 0.0);
      m_txReef = m_netTableReef->GetNumber("tx", 0.0);
      m_tidReef = m_netTableReef->GetNumber("tid", 0);
      m_logtxReef.Append(m_txReef);
      m_logtyReef.Append(m_tyReef);
      m_logtidReef.Append(m_tidReef);
  }
  // else
  // {
  // }
}


void VisionSubsystem::SetLoadPositionPipeline() 
{
  m_netTableLoad->PutNumber("pipeline", c_limelightLoadPositionPipeline);
}

void VisionSubsystem::SetLoadAnglePipeline() 
{
  m_netTableLoad->PutNumber("pipeline", c_limelightLoadAnglePipeline);
}