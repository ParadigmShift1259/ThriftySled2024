#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"


class CoralEjectCommand: public frc2::CommandHelper<frc2::Command, CoralEjectCommand>
{
    public:
        explicit CoralEjectCommand(ISubsystemAccess& subsystemAccess);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        CoralManipulatorSubsystem&  m_coralSubsystem;
#ifdef LED
        LEDSubsystem&               m_ledSubsystem;
#endif

		wpi::log::BooleanLogEntry   m_logStartCommand;
};