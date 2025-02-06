
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
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kPreCoral);
    m_ledSubsystem.SetAnimation(c_colorPink, LEDSubsystem::kStrobe);  //TODO Replace constant color with var based on left/right & Set height based on level
#endif
    m_elevatorSubsystem.GoToPosition(m_coralLevel);
    m_coralEncPos = m_coralSubsystem.GetPosition() + turns;
    m_retract = true;
}

void CoralPrepCommand::Execute()
{
    if (m_elevatorSubsystem.IsAtPosition(m_coralLevel) && m_retract)
    {
        // m_coralSubsystem.SetManipulator(0.5); This is for L4 only goes all the way back
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
        m_coralSubsystem.RetractCoral(eLevel);
        m_retract = false;
    }

}

bool CoralPrepCommand::IsFinished()
{
    // return m_coralSubsystem.IsCoralPresentOutput() == false; for L4 only
    return fabs(m_coralSubsystem.GetPosition() - m_coralEncPos) <= 0.5 || m_timer.HasElapsed(2.0_s);
}

void CoralPrepCommand::End(bool interrupted)
{
    m_coralSubsystem.Stop();
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kIdle);
#endif
}