#include "commands/ClimbDeployCommand.h"

ClimbDeployCommand::ClimbDeployCommand(ISubsystemAccess& subsystemAccess)
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

void ClimbDeployCommand::Initialize()
{
  m_logStartCommand.Append(true);

#ifdef LED
  m_led.SetCurrentAction(LEDSubsystem::kClimbing);
  m_led.SetAnimation(c_colorGreen, LEDSubsystem::kFlow);
#endif
  m_climbSubsystem.GoToPosition(c_defaultClimbDeployTurns);
}

void ClimbDeployCommand::Execute()
{
}

bool ClimbDeployCommand::IsFinished()
{
  return (fabs(m_climbSubsystem.GetPosition() - c_defaultClimbDeployTurns) <= 20.0);
}

void ClimbDeployCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
#ifdef LED
  m_led.SetAnimation(c_colorGreen, LEDSubsystem::kSolid);
  m_led.SetCurrentAction(LEDSubsystem::kClimbFinish);
#endif
}
