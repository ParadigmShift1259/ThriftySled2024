#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ResetAngleAtLoadCommand : public frc2::CommandHelper<frc2::Command, ResetAngleAtLoadCommand>
{
public:
    explicit ResetAngleAtLoadCommand(ISubsystemAccess& subsystemAccess);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    DriveSubsystem& m_drive;

    wpi::log::BooleanLogEntry m_logStartCommand;
};
