#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"


class CoralPrepCommand: public frc2::CommandHelper<frc2::Command, CoralPrepCommand>
{
    public:
        explicit CoralPrepCommand(ISubsystemAccess& subsystemAccess, double coralLevel);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:

        CoralManipulatorSubsystem&        m_coralSubsystem;
        ElevatorSubsystem&                m_elevatorSubsystem;

        frc::Timer m_timer;

        double m_coralLevel = c_defaultL1Turns;
        double m_coralEncPos = 0.0;
        bool m_retract = true;

		wpi::log::BooleanLogEntry m_logStartCoralPrepCommand;
		wpi::log::BooleanLogEntry m_logCoralPrepCommandFlipped;
};