
#include "commands/CoralPrepCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralPrepCommand::CoralPrepCommand(ISubsystemAccess& subsystemAccess)
    : m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevatorSubsystem(subsystemAccess.GetElevator())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
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
    m_coralLevel = m_elevatorSubsystem.GetPresetLevel();
    m_blocked = (m_coralLevel == L4 || m_coralLevel == L3) && m_coralSubsystem.IsCoralPresentInput();
    if (m_blocked)
    {
        return;
    }
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kSequenceStart);
    m_ledSubsystem.SetAnimation(c_colorBlue, LEDSubsystem::kFlow);  //TODO Replace constant color with var based on left/right & Set height based on level
#endif

    if (m_coralLevel == L4)
    {
        m_elevatorSubsystem.GoToPosition(37.0); // Anti-slam, subsequent command should move it to L4
    }
    else
    {
        m_elevatorSubsystem.GoToPresetLevel();
    }
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
    return m_blocked || fabs(m_coralSubsystem.GetPosition() - m_coralEncPos) <= 1.0 || m_timer.HasElapsed(2.0_s);
}

void CoralPrepCommand::End(bool interrupted)
{
    m_coralSubsystem.Stop();
    m_logStartCommand.Append(false);
}