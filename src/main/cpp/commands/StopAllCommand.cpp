#include "commands/StopAllCommand.h"

StopAllCommand::StopAllCommand(ISubsystemAccess& subsystemAccess)
    : m_intakeSubsystem(subsystemAccess.GetIntake())
    , m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevSubsystem(subsystemAccess.GetElevator())
    //m_climbSubsystem(subsystemAccess.GetClimber())
#ifdef LED
    , m_led(subsystemAccess.GetLED())
#endif
    , m_drive(subsystemAccess.GetDrive())
    , m_vision(subsystemAccess.GetVision())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetIntake()
                                    , &subsystemAccess.GetCoral()
                                    , &subsystemAccess.GetElevator()
                                    //&subsystemAccess.GetClimber())
#ifdef LED
                                    , &subsystemAccess.GetLED()
#endif
                                    , &subsystemAccess.GetVision()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/StopAllCommand/startCommand");
}

void StopAllCommand::Initialize()
{
  m_logStartCommand.Append(true);

  m_intakeSubsystem.Stop();
  m_coralSubsystem.Stop();
  m_elevSubsystem.Stop();
  //m_climbSubsystem(subsystemAccess.GetClimber())
  m_drive.Stop();
#ifdef LED
  // m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  // m_led.SetCurrentAction(LEDSubsystem::ECurrentAction::kIdle);
#endif
  // TODO scheduler cancel???
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
