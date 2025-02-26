#include "commands/ClimbRetractCommand.h"

ClimbRetractCommand::ClimbRetractCommand(ISubsystemAccess& subsystemAccess)
    : m_climbSubsystem(subsystemAccess.GetClimber())
#ifdef LED
    , m_led(subsystemAccess.GetLED())
#endif
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetClimber()
#ifdef LED
                                     , &subsystemAccess.GetLED()
#endif
    });

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
}

void ClimbRetractCommand::Initialize()
{
  m_logStartCommand.Append(true);

#ifdef LED
  m_led.SetAnimation(c_colorBlue, LEDSubsystem::kFlow);
#endif
  m_climbSubsystem.GoToPosition(c_defaultClimbResetTurns);
}

void ClimbRetractCommand::Execute()
{
}

bool ClimbRetractCommand::IsFinished()
{
  return (fabs(m_climbSubsystem.GetPosition() - c_defaultClimbResetTurns) <= 20.0);
}

void ClimbRetractCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
#ifdef LED
  m_led.SetAnimation(c_colorBlue, LEDSubsystem::kSolid);
  m_led.SetCurrentAction(LEDSubsystem::kClimbFinish);
#endif
}
