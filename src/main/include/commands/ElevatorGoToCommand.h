#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"


class ElevatorGoToCommand: public frc2::CommandHelper<frc2::Command, ElevatorGoToCommand>
{
    public:
        explicit ElevatorGoToCommand(ISubsystemAccess& subsystemAccess, double position);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:

        ElevatorSubsystem&        m_elevatorSubsystem;
#ifdef LED
        LEDSubsystem&                     m_ledSubsystem;
#endif

        double m_position = 0.0;

        frc::Timer m_timer;

		wpi::log::BooleanLogEntry m_logStartElevatorGoToCommand;
		wpi::log::BooleanLogEntry m_logElevatorGoToCommandFlipped;
};
