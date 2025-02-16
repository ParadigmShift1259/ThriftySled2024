#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"


class CoralPrepCommand: public frc2::CommandHelper<frc2::Command, CoralPrepCommand>
{
    public:
        explicit CoralPrepCommand(ISubsystemAccess& subsystemAccess, ELevels coralLevel);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        CoralManipulatorSubsystem&        m_coralSubsystem;
        ElevatorSubsystem&                m_elevatorSubsystem;
#ifdef LED
        LEDSubsystem&                     m_ledSubsystem;
#endif

        frc::Timer m_timer;

        ELevels m_coralLevel = L1;
        double m_coralEncPos = 0.0;
        bool m_retract = true;
        bool m_blocked = false;

		wpi::log::BooleanLogEntry m_logStartCoralPrepCommand;
		wpi::log::BooleanLogEntry m_logCoralPrepCommandFlipped;
};