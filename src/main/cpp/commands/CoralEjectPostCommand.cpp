
#include "commands/CoralEjectPostCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralEjectPostCommand::CoralEjectPostCommand(ISubsystemAccess& subsystemAccess)
    : m_elevatorSubsystem(subsystemAccess.GetElevator())
    , m_driveSubsystem(subsystemAccess.GetDrive())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
{
#ifdef LED
    AddRequirements(frc2::Requirements{&subsystemAccess.GetElevator(), &subsystemAccess.GetDrive(), &subsystemAccess.GetLED()});
#else
    AddRequirements(frc2::Requirements{&subsystemAccess.GetElevator(), &subsystemAccess.GetDrive()});
#endif
    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCoralEjectPostCommand = wpi::log::BooleanLogEntry(log, "/CoralEjectPostCommand/startCommand");
    m_logCoralEjectPostCommandFlipped = wpi::log::BooleanLogEntry(log, "/CoralEjectPostCommand/startCommand");

}

void CoralEjectPostCommand::Initialize()
{
    printf("Coral Post Initialized \n");
    m_timer.Reset();
    m_timer.Start();
    m_driveSubsystem.WheelsForward();
}

void CoralEjectPostCommand::Execute()
{
    if(m_timer.HasElapsed(0.5_s))
    {
        m_elevatorSubsystem.GoToPosition(c_defaultL2Turns);
    }
}

bool CoralEjectPostCommand::IsFinished()
{
    return m_timer.HasElapsed(1.0_s);
}

void CoralEjectPostCommand::End(bool interrupted)
{
    m_elevatorSubsystem.Stop();
    m_driveSubsystem.Stop();
    printf("Coral Post End \n");
}