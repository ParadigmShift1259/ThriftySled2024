
#include "commands/CoralPrepCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralPrepCommand::CoralPrepCommand(ISubsystemAccess& subsystemAccess, double coralLevel)
    : m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevatorSubsystem(subsystemAccess.GetElevator())
    , m_coralLevel(coralLevel)
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetElevator()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCoralPrepCommand = wpi::log::BooleanLogEntry(log, "/CoralPrepCommand/startCommand");
    m_logCoralPrepCommandFlipped = wpi::log::BooleanLogEntry(log, "/CoralPrepCommand/startCommand");

}

void CoralPrepCommand::Initialize()
{
    auto turns = frc::SmartDashboard::GetNumber("CoralRetractTurns", 3.25);
    m_timer.Reset();
    m_timer.Start();
    m_elevatorSubsystem.GoToPosition(m_coralLevel);
    m_coralEncPos = m_coralSubsystem.GetPosition() + turns;
    m_retract = true;
}

void CoralPrepCommand::Execute()
{
    if (m_elevatorSubsystem.IsAtPosition(m_coralLevel) && m_retract)
    {
        // m_coralSubsystem.SetManipulator(0.5); This is for L4 only goes all the way back
        m_coralSubsystem.RetractCoral();
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
    printf("Coral Prep Ended \n");
}