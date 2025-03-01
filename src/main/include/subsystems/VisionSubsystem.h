#pragma once

#include <vector>
#include <wpi/DataLog.h>
#include <wpi/interpolating_map.h>

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/filter/LinearFilter.h>
#include <frc/DataLogManager.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <iostream>

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
    bool IsValidLoad() { return m_isValidLoad; }
    bool IsValidReef() { return m_isValidReef; }
    double GetX() { return m_netBufferAlli[eX]; }
    double GetY() { return m_netBufferAlli[eY]; }
    double GetYaw() { return m_netBufferAlli[eYaw]; }
    double GetLatency() { return m_netBufferAlli[eLatency]; }
    int GetTagId() { return m_tidReef; }
    void EnableLoadLEDs() { m_netTableLoad->PutNumber("ledMode", 1); } //3); }
    void DisableLoadLEDs() { m_netTableLoad->PutNumber("ledMode", 1); }
    void EnableReefLEDs() { m_netTableReef->PutNumber("ledMode", 3); }
    void DisableReefLEDs() { m_netTableReef->PutNumber("ledMode", 1); }
    double GetYawError() { return m_yawError; }
    void SetLoadPositionPipeline();
    void SetLoadAnglePipeline();
    double GetFloorDist() { return m_floorDistance; };
    void ToggleAllowedReef() { m_isAllowedReef = !m_isAllowedReef; }
    void ToggleAllowedLoad() { m_isAllowedLoad = !m_isAllowedLoad; }
    bool IsPositionStarted() { return m_positionStarted; }
    void SetPositionStarted(bool bVal) { m_positionStarted = bVal; }
    bool IsAzimuthStarted() { return m_azimuthStarted; }
    void SetAzimuthStarted(bool bVal) { m_azimuthStarted = bVal; }

  private:
    void PeriodicLoad();
    void PeriodicReef();

    bool m_isValidLoad = false;
    bool m_isValidReef = false;
    bool m_isAllowedLoad = true;
    bool m_isAllowedReef = true;
    bool m_azimuthStarted = false;
    bool m_positionStarted = false;
    std::vector<double> m_netBufferField{2};
    std::vector<double> m_netBufferAlli{2};
    std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

    std::shared_ptr<nt::NetworkTable> m_netTableLoad = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-load");

    std::shared_ptr<nt::NetworkTable> m_netTableReef = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-reef");
  
  bool m_bIsBlue = false;
  double m_tyLoad = 0.0;
  double m_txLoad = 0.0;
  double m_tyReef = 0.0;
  double m_txReef = 0.0;
  int m_tidReef = -1;
  double m_floorDistance = 0.0;
  double m_yawError = 0.0;
  double m_commandedAzimuth = 0.0;


  double c_defaultAimP = -0.1;
  double c_minAimCommanded = 0.05;



  wpi::log::DoubleLogEntry m_logRobotAlliPoseX;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseY;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseTheta;
  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
  wpi::log::DoubleLogEntry m_logLL_Latency;
  wpi::log::DoubleLogEntry m_logtxLoad;
  wpi::log::DoubleLogEntry m_logtyLoad;
  wpi::log::DoubleLogEntry m_logtxReef;
  wpi::log::DoubleLogEntry m_logtyReef;
  wpi::log::IntegerLogEntry m_logtidReef;
};