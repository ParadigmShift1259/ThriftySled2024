#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class StopAllCommand: public frc2::CommandHelper<frc2::Command, StopAllCommand>
{
public:
    explicit StopAllCommand(ISubsystemAccess& subsystemAccess);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    IntakeSubsystem& m_intakeSubsystem;
    CoralManipulatorSubsystem& m_coralSubsystem;
    ElevatorSubsystem& m_elevSubsystem;
    //ClimberSubsystem& m_climbSubsystem;
    DriveSubsystem& m_drive;
    VisionSubsystem& m_vision;

    wpi::log::BooleanLogEntry m_logStartCommand;
};
