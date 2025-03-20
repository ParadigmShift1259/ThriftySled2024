#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "subsystems/VisionSubsystem.h"

constexpr double c_limelightReefMountAngle = 30.0;

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
  m_logtxReef = wpi::log::DoubleLogEntry(log, "/vision/txReef");
  m_logtyReef = wpi::log::DoubleLogEntry(log, "/vision/tyReef");
  m_logtidReef = wpi::log::IntegerLogEntry(log, "/vision/tidReef");

  // frc::SmartDashboard::PutBoolean("AllowedReef", m_isAllowedReef);
}

void VisionSubsystem::Periodic()
{
//  PeriodicLoad();
  PeriodicReef();
  // m_isAllowedReef = true;//frc::SmartDashboard::GetBoolean("AllowedReef", m_isAllowedReef);
}

void VisionSubsystem::PeriodicReef()
{
  m_isValidReef = m_netTableReef->GetNumber("tv", 0) == 1.0;
  frc::SmartDashboard::PutNumber("tv Reef", m_isValidReef);
  // if (m_isValidReef && m_isAllowedReef)
  if (m_isValidReef)
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

      m_dbvTag.Put(m_tidReef);
  }
  // else
  // {
  // }
}
