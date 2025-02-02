
#include "commands/CoralEjectCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralEjectCommand::CoralEjectCommand(ISubsystemAccess& subsystemAccess)
    : m_coralSubsystem(subsystemAccess.GetCoral())
    , m_elevatorSubsystem(subsystemAccess.GetElevator())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
{
#ifdef LED
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetElevator(), &subsystemAccess.GetLED()});
#else
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetElevator()});
#endif
    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCoralEjectCommand = wpi::log::BooleanLogEntry(log, "/CoralEjectCommand/startCommand");
    m_logCoralEjectCommandFlipped = wpi::log::BooleanLogEntry(log, "/CoralEjectCommand/startCommand");

}

void CoralEjectCommand::Initialize()
{
#ifdef LED
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kDefaultAction);
    m_ledSubsystem.SetAnimation(c_defaultColor, LEDSubsystem::kSolid);
#endif
    m_timer.Reset();
    m_timer.Start();

}

void CoralEjectCommand::Execute()
{
    m_coralSubsystem.EjectCoral(false);
}

bool CoralEjectCommand::IsFinished()
{
    return m_coralSubsystem.IsCoralPresentOutput() == false;
}

void CoralEjectCommand::End(bool interrupted)
{
    m_coralSubsystem.Stop();
    printf("Coral Eject End \n");
}