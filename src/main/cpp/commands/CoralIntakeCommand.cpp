
#include "commands/CoralIntakeCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralIntakeCommand::CoralIntakeCommand(ISubsystemAccess& subsystemAccess)
    : m_coralSubsystem(subsystemAccess.GetCoral())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCoralIntakeCommand = wpi::log::BooleanLogEntry(log, "/CoralIntakeCommand/startCommand");
    m_logCoralIntakeCommandFlipped = wpi::log::BooleanLogEntry(log, "/CoralIntakeCommand/startCommand");

}

void CoralIntakeCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
    m_coralSubsystem.SetFeeder(0.5);
}

void CoralIntakeCommand::Execute()
{
    if (m_coralSubsystem.IsCoralPresentInput()){
        m_isCoralPresent = true;
        m_coralSubsystem.SetManipulator(-0.5);
    }
}

bool CoralIntakeCommand::IsFinished()
{
    return m_isCoralPresent == true && m_coralSubsystem.IsCoralPresentInput() == false;
}

void CoralIntakeCommand::End(bool interrupted)
{
    m_isCoralPresent = false;
    m_coralSubsystem.Stop();
}
