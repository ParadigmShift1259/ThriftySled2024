
#include "commands/CoralEjectPostCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

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
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/CoralEjectPostCommand/startCommand");
}

void CoralEjectPostCommand::Initialize()
{
    m_logStartCommand.Append(true);
    m_timer.Reset();
    m_timer.Start();

    // if (frc::DriverStation::IsTeleopEnabled())
    // {
    //     m_driveSubsystem.DriveBack();
    // }
}

void CoralEjectPostCommand::Execute()
{
    if(m_timer.HasElapsed(0.25_s)) // Waiting to get robot clear of reef
    {
        m_elevatorSubsystem.GoToPosition(L1);
    }
}

bool CoralEjectPostCommand::IsFinished()
{
    if (frc::DriverStation::IsAutonomousEnabled())
    {
        return m_elevatorSubsystem.GetLowerLimit() || m_timer.HasElapsed(0.25_s);
    }
    else
    {
        return m_elevatorSubsystem.GetLowerLimit() || m_timer.HasElapsed(0.75_s);
    }
}

void CoralEjectPostCommand::End(bool interrupted)
{
    // Don't need to do this, the limit switch should turn off the motor m_elevatorSubsystem.Stop();
    if (frc::DriverStation::IsTeleopEnabled())
    {
        m_driveSubsystem.Stop();
    }
    m_logStartCommand.Append(false);
}