
#include "commands/CoralEjectCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralEjectCommand::CoralEjectCommand(ISubsystemAccess& subsystemAccess)
    : m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevatorSubsystem(subsystemAccess.GetElevator())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetElevator()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCoralEjectCommand = wpi::log::BooleanLogEntry(log, "/CoralEjectCommand/startCommand");
    m_logCoralEjectCommandFlipped = wpi::log::BooleanLogEntry(log, "/CoralEjectCommand/startCommand");

}

void CoralEjectCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
    m_elevatorSubsystem.GoToPosition(c_defaultL4Turns);
}

void CoralEjectCommand::Execute()
{
    if (m_elevatorSubsystem.IsAtPosition(c_defaultL4Turns)){
        m_coralSubsystem.SetManipulator(0.5);
    }
}

bool CoralEjectCommand::IsFinished()
{
    return m_coralSubsystem.IsCoralPresentOutput();
}

void CoralEjectCommand::End(bool interrupted)
{
    m_isCoralPresent = false;
    m_coralSubsystem.Stop();
}