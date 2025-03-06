#pragma once

#include <vector>

#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DataLogManager.h>

// #include <units/angle.h>
// #include <units/angular_velocity.h>
// #include <units/length.h>

#include "DashBoardValue.h"

enum BotPoseIndices
{
    eX
  , eY
  , eZ
  , ePitch
  , eRoll
  , eYaw
  , eLatency
};

class VisionSubsystem : public frc2::SubsystemBase
{
  public:
    VisionSubsystem();
    
    void Periodic() override;
    bool IsValidReef() { return m_isValidReef; }
    double GetX() { return m_netBufferAlli[eX]; }
    double GetY() { return m_netBufferAlli[eY]; }
    double GetYaw() { return m_netBufferAlli[eYaw]; }
    double GetLatency() { return m_netBufferAlli[eLatency]; }
    int GetTagId() { return m_tidReef; }
    void EnableReefLEDs() { m_netTableReef->PutNumber("ledMode", 3); }
    void DisableReefLEDs() { m_netTableReef->PutNumber("ledMode", 1); }
    void ToggleAllowedReef() { m_isAllowedReef = !m_isAllowedReef; }

  private:
    void PeriodicReef();

    bool m_isValidReef = false;
    bool m_isAllowedReef = true;
    std::vector<double> m_netBufferField{2};
    std::vector<double> m_netBufferAlli{2};
    std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

    std::shared_ptr<nt::NetworkTable> m_netTableReef = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-reef");

  DashBoardValue<double> m_dbvTag{"Vision", "Tag", -1.0};
  
  bool m_bIsBlue = false;
  double m_tyReef = 0.0;
  double m_txReef = 0.0;
  int m_tidReef = -1;

  wpi::log::DoubleLogEntry m_logRobotAlliPoseX;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseY;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseTheta;
  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
  wpi::log::DoubleLogEntry m_logLL_Latency;
  wpi::log::DoubleLogEntry m_logtxReef;
  wpi::log::DoubleLogEntry m_logtyReef;
  wpi::log::IntegerLogEntry m_logtidReef;
};