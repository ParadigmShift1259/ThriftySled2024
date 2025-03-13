
#include "commands/CoralPrepCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralPrepCommand::CoralPrepCommand(ISubsystemAccess& subsystemAccess, ELevels coralLevel)
    : m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevatorSubsystem(subsystemAccess.GetElevator())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
    , m_coralLevel(coralLevel)
{
#ifdef LED
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetElevator(), &subsystemAccess.GetLED()});
#else
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetElevator(), });
#endif

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/CoralPrepCommand/startCommand");
}

void CoralPrepCommand::Initialize()
{
    m_logStartCommand.Append(true);
    auto turns = frc::SmartDashboard::GetNumber("CoralRetractTurns", 3.25);
    m_timer.Reset();
    m_timer.Start();
    m_blocked = (m_coralLevel == L4 || m_coralLevel == L3) && m_coralSubsystem.IsCoralPresentInput();
    if (m_blocked)
    {
        return;
    }
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kPreCoral);
    m_ledSubsystem.SetAnimation(c_colorPink, LEDSubsystem::kStrobe);  //TODO Replace constant color with var based on left/right & Set height based on level
#endif

    m_elevatorSubsystem.GoToPresetLevel();
    m_coralLevel = m_elevatorSubsystem.GetPresetLevel();
    m_coralEncPos = m_coralSubsystem.GetPosition() + turns;
    m_retract = true;
}

void CoralPrepCommand::Execute()
{
    if (m_retract)
    {
        m_coralSubsystem.RetractCoral(m_coralLevel);
        m_retract = false;
    }
}

bool CoralPrepCommand::IsFinished()
{   
    return m_blocked || fabs(m_coralSubsystem.GetPosition() - m_coralEncPos) <= 0.5 || m_timer.HasElapsed(2.0_s);
}

void CoralPrepCommand::End(bool interrupted)
{
    m_coralSubsystem.Stop();
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kIdle);
#endif
    m_logStartCommand.Append(false);
}