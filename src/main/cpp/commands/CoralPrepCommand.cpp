
#include "commands/CoralPrepCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralPrepCommand::CoralPrepCommand(ISubsystemAccess& subsystemAccess)
    : m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevatorSubsystem(subsystemAccess.GetElevator())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetElevator()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCoralPrepCommand = wpi::log::BooleanLogEntry(log, "/CoralPrepCommand/startCommand");
    m_logCoralPrepCommandFlipped = wpi::log::BooleanLogEntry(log, "/CoralPrepCommand/startCommand");

}

void CoralPrepCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
    m_elevatorSubsystem.GoToPosition(c_defaultL4Turns);
}

void CoralPrepCommand::Execute()
{
    if (m_elevatorSubsystem.IsAtPosition(c_defaultL3Turns)){
        m_coralSubsystem.SetManipulator(0.5);
    }
}

bool CoralPrepCommand::IsFinished()
{
    return m_coralSubsystem.IsCoralPresentOutput() == false;
}

void CoralPrepCommand::End(bool interrupted)
{
    m_coralSubsystem.Stop();
}