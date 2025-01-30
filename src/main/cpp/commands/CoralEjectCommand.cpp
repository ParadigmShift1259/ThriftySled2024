
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
    printf("Coral Eject Initialized \n");
    m_timer.Reset();
    m_timer.Start();
}

void CoralEjectCommand::Execute()
{
    m_coralSubsystem.EjectCoral();
}

bool CoralEjectCommand::IsFinished()
{
    return m_coralSubsystem.IsCoralPresentOutput() == false;
}

void CoralEjectCommand::End(bool interrupted)
{
    m_coralSubsystem.Stop();
    printf("Coral Eject End \n");
}