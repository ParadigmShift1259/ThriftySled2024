#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"


class CoralEjectPostCommand: public frc2::CommandHelper<frc2::Command, CoralEjectPostCommand>
{
    public:
        explicit CoralEjectPostCommand(ISubsystemAccess& subsystemAccess);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ElevatorSubsystem&                m_elevatorSubsystem;
        DriveSubsystem&                   m_driveSubsystem;

        frc::Timer m_timer;

		wpi::log::BooleanLogEntry m_logStartCoralEjectPostCommand;
		wpi::log::BooleanLogEntry m_logCoralEjectPostCommandFlipped;
};