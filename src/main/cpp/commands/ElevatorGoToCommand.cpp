
#include "commands/ElevatorGoToCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

ElevatorGoToCommand::ElevatorGoToCommand(ISubsystemAccess& subsystemAccess, ELevels eLevel, bool bUsePresetLevel /* = false */)
    : m_elevatorSubsystem(subsystemAccess.GetElevator())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
    , m_level(eLevel)
    , m_bUsePresetLevel(bUsePresetLevel)
{
#ifdef LED
    AddRequirements(frc2::Requirements{&subsystemAccess.GetElevator(), &subsystemAccess.GetLED()});
#else
    AddRequirements(frc2::Requirements{&subsystemAccess.GetElevator()});
#endif

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ElevatorGoToCommand/startCommand");
}

void ElevatorGoToCommand::Initialize()
{
    m_logStartCommand.Append(true);
    m_timer.Reset();
    m_timer.Start();
}

void ElevatorGoToCommand::Execute()
{
    if (m_bUsePresetLevel)
    {
        //printf("ElevatorGoToCommand going to preset level %d\n", m_level + 1);
        m_elevatorSubsystem.GoToPresetLevel();
    }
    else
    {
        m_elevatorSubsystem.GoToPosition(m_level);
    }
}

bool ElevatorGoToCommand::IsFinished()
{
    return true;
}

void ElevatorGoToCommand::End(bool interrupted)
{
    m_logStartCommand.Append(false);
}
