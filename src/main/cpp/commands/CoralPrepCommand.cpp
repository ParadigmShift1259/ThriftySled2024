
#include "commands/CoralPrepCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralPrepCommand::CoralPrepCommand(ISubsystemAccess& subsystemAccess, double coralLevel)
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
    m_logStartCoralPrepCommand = wpi::log::BooleanLogEntry(log, "/CoralPrepCommand/startCommand");
    m_logCoralPrepCommandFlipped = wpi::log::BooleanLogEntry(log, "/CoralPrepCommand/startCommand");

}

void CoralPrepCommand::Initialize()
{
    auto turns = frc::SmartDashboard::GetNumber("CoralRetractTurns", 3.25);
    m_timer.Reset();
    m_timer.Start();
    m_blocked = (m_coralLevel == c_defaultL4Turns || m_coralLevel == c_defaultL3Turns) && m_coralSubsystem.IsCoralPresentInput();
    if (m_blocked)
    {
        return;
    }
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kPreCoral);
    m_ledSubsystem.SetAnimation(c_colorPink, LEDSubsystem::kStrobe);  //TODO Replace constant color with var based on left/right & Set height based on level
#endif
#ifdef PRACTICE_BINDINGS
    m_elevatorSubsystem.GoToPosition(m_coralLevel);
#else
    m_elevatorSubsystem.GoToPresetLevel();
    ELevels eLevel = m_elevatorSubsystem.GetPresetLevel();
    switch (eLevel)
    {
        case L1:
            m_coralLevel = c_defaultL1Turns;
            break;
        case L2:
            m_coralLevel = c_defaultL2Turns;
            break;
        case L3:
            m_coralLevel = c_defaultL3Turns;
            break;
        case L4:
            m_coralLevel = c_defaultL4Turns;
            break;
        default:
            break;
    }
        
#endif
    m_coralEncPos = m_coralSubsystem.GetPosition() + turns;
    m_retract = true;
}

void CoralPrepCommand::Execute()
{
    if (m_elevatorSubsystem.IsAtPosition(m_coralLevel) && m_retract)
    {
        // m_coralSubsystem.SetManipulator(0.5); This is for L4 only goes all the way back
#ifdef PRACTICE_BINDINGS
        ELevels eLevel = L1;
        if (m_coralLevel == c_defaultL2Turns)
        {
            eLevel = L2;
        }
        else if (m_coralLevel == c_defaultL3Turns)
        {
            eLevel = L3;
        }
        else if (m_coralLevel == c_defaultL4Turns)
        {
            eLevel = L4;
        }
#else
        ELevels eLevel = m_elevatorSubsystem.GetPresetLevel();
#endif
        m_coralSubsystem.RetractCoral(eLevel);
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
}