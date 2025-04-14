#include "commands/StopAllCommand.h"
#include <frc2/command/CommandScheduler.h>

StopAllCommand::StopAllCommand(ISubsystemAccess& subsystemAccess)
    : m_intakeSubsystem(subsystemAccess.GetIntake())
    , m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevSubsystem(subsystemAccess.GetElevator())
    //m_climbSubsystem(subsystemAccess.GetClimber())
    , m_drive(subsystemAccess.GetDrive())
    , m_vision(subsystemAccess.GetVision())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetIntake()
                                    , &subsystemAccess.GetCoral()
                                    , &subsystemAccess.GetElevator()
                                    //&subsystemAccess.GetClimber())
                                    , &subsystemAccess.GetDrive()
                                    , &subsystemAccess.GetVision()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/StopAllCommand/startCommand");
}

void StopAllCommand::Initialize()
{
  m_logStartCommand.Append(true);

  frc2::CommandScheduler::GetInstance().CancelAll();

  m_intakeSubsystem.Stop();
  m_coralSubsystem.Stop();
  m_elevSubsystem.Stop();
  //m_climbSubsystem(subsystemAccess.GetClimber())
  m_drive.Stop();
}

void StopAllCommand::Execute()
{
}

bool StopAllCommand::IsFinished()
{
  return true;
}

void StopAllCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}
