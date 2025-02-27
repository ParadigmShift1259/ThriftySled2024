
#include "commands/CoralEjectCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

CoralEjectCommand::CoralEjectCommand(ISubsystemAccess& subsystemAccess)
    : m_coralSubsystem(subsystemAccess.GetCoral())
#ifdef LED
    , m_ledSubsystem(subsystemAccess.GetLED())
#endif
{
#ifdef LED
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral(), &subsystemAccess.GetLED()});
#else
    AddRequirements(frc2::Requirements{&subsystemAccess.GetCoral()});
#endif
    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/CoralEjectCommand/startCommand");
}

void CoralEjectCommand::Initialize()
{
    m_logStartCommand.Append(true);
    m_coralSubsystem.EjectCoral(false);
#ifdef LED 
    m_ledSubsystem.SetCurrentAction(LEDSubsystem::kDefaultAction);
    m_ledSubsystem.SetAnimation(c_defaultColor, LEDSubsystem::kSolid);
#endif
}

void CoralEjectCommand::Execute()
{
}

bool CoralEjectCommand::IsFinished()
{
    return m_coralSubsystem.IsCoralPresentOutput() == false;
}

void CoralEjectCommand::End(bool interrupted)
{
    m_coralSubsystem.RetractManipulator();
    m_coralSubsystem.Stop(); //Stops coral eject motors
    m_logStartCommand.Append(false);
}