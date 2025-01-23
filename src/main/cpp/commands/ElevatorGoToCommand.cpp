
#include "commands/ElevatorGoToCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

ElevatorGoToCommand::ElevatorGoToCommand(ISubsystemAccess& subsystemAccess, double position)
    : m_elevatorSubsystem(subsystemAccess.GetElevator())
    , m_position(position)
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetElevator()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartElevatorGoToCommand = wpi::log::BooleanLogEntry(log, "/ElevatorGoToCommand/startCommand");
    m_logElevatorGoToCommandFlipped = wpi::log::BooleanLogEntry(log, "/ElevatorGoToCommand/startCommand");

}

void ElevatorGoToCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
}

void ElevatorGoToCommand::Execute()
{
    m_elevatorSubsystem.GoToPosition(m_position);
}

bool ElevatorGoToCommand::IsFinished()
{
    return true;
}

void ElevatorGoToCommand::End(bool interrupted)
{

}
