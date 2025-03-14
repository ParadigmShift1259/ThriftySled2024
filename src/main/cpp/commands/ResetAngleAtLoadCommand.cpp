#include "commands/ResetAngleAtLoadCommand.h"
#include <frc/DriverStation.h>

ResetAngleAtLoadCommand::ResetAngleAtLoadCommand(ISubsystemAccess& subsystemAccess)
    : m_drive(subsystemAccess.GetDrive())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ResetAngleAtLoadCommand/startCommand");
}

void ResetAngleAtLoadCommand::Initialize()
{
  m_logStartCommand.Append(true);

  constexpr Rotation2d c_loadStationBlueAngle{54_deg};
  constexpr Rotation2d c_loadStationRedAngle{126_deg};
  constexpr units::length::meter_t c_centerLineY = 158.50_in;

  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance)
  {
    // Assume blue
    Rotation2d loadStationAngle = c_loadStationBlueAngle;
    bool bAboveCenterLine = (m_drive.GetY() > c_centerLineY);
    if (bAboveCenterLine)
    {
      loadStationAngle = -c_loadStationBlueAngle;
    }

    if (alliance.value() == frc::DriverStation::Alliance::kRed)
    {
      loadStationAngle = c_loadStationRedAngle;
      if (bAboveCenterLine)
      {
        loadStationAngle = -c_loadStationRedAngle;
      }
    }
    printf("Resetting pose angle at load station to %.3f\n", loadStationAngle.Degrees().value());
    Pose2d resetPose{m_drive.GetX(), m_drive.GetY(), loadStationAngle};
    m_drive.ResetOdometry(resetPose);
  }
  else
  {
    printf("Could not determine alliance - skipping reset pose angle at load station\n");
  }

}

void ResetAngleAtLoadCommand::Execute()
{
}

bool ResetAngleAtLoadCommand::IsFinished()
{
  return true;
}

void ResetAngleAtLoadCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}
