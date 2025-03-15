#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"

constexpr bool c_bUsePresetLevel = true;

class ElevatorGoToCommand: public frc2::CommandHelper<frc2::Command, ElevatorGoToCommand>
{
    public:
        explicit ElevatorGoToCommand(ISubsystemAccess& subsystemAccess, ELevels eLevel, bool bUsePresetLevel = false);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:

        ElevatorSubsystem&        m_elevatorSubsystem;
#ifdef LED
        LEDSubsystem&                     m_ledSubsystem;
#endif

        ELevels m_level = L1;

        frc::Timer m_timer;
        bool m_bUsePresetLevel = false;

		wpi::log::BooleanLogEntry m_logStartCommand;
};
