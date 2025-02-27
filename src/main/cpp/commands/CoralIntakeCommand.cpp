
#include "commands/CoralIntakeCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralIntakeCommand::CoralIntakeCommand(ISubsystemAccess& subsystemAccess)
    : m_coralSubsystem(subsystemAccess.GetCoral())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
{
#ifdef LED
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetLED()});
#else
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral()});
#endif
    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/CoralIntakeCommand/startCommand");
}

void CoralIntakeCommand::Initialize()
{
    m_logStartCommand.Append(true);
    m_timer.Reset();
    m_timer.Start();
    m_coralSubsystem.SetManipulator(-0.5);
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kHasCoral);
#endif
}

void CoralIntakeCommand::Execute()
{
    if (m_coralSubsystem.IsCoralPresentOutput())
    {
#ifdef LED
        m_ledSubsystem.SetAnimation(c_colorPink, LEDSubsystem::kSolid);
#endif
    }
}

bool CoralIntakeCommand::IsFinished()
{
    return m_coralSubsystem.IsCoralPresentOutput() == true && m_coralSubsystem.IsCoralPresentInput() == false;
}

void CoralIntakeCommand::End(bool interrupted)
{
    m_coralSubsystem.Stop();
    m_logStartCommand.Append(false);
}
