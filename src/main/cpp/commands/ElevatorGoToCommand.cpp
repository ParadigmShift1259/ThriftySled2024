
#include "commands/ElevatorGoToCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

ElevatorGoToCommand::ElevatorGoToCommand(ISubsystemAccess& subsystemAccess, ELevels eLevel)
    : m_elevatorSubsystem(subsystemAccess.GetElevator())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
    , m_level(eLevel)
{
#ifdef LED
    AddRequirements(frc2::Requirements{&subsystemAccess.GetElevator(), &subsystemAccess.GetLED()});
#else
    AddRequirements(frc2::Requirements{&subsystemAccess.GetElevator()});
#endif

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
    m_elevatorSubsystem.GoToPosition(m_level);
}

bool ElevatorGoToCommand::IsFinished()
{
    return true;
}

void ElevatorGoToCommand::End(bool interrupted)
{

}
