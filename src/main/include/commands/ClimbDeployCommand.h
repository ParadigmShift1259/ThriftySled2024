#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ClimbDeployCommand: public frc2::CommandHelper<frc2::Command, ClimbDeployCommand>
{
public:
    explicit ClimbDeployCommand(ISubsystemAccess& subsystemAccess);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ClimberSubsystem& m_climbSubsystem;
#ifdef LED
    LEDSubsystem& m_led;
#endif
    wpi::log::BooleanLogEntry m_logStartCommand;
};
