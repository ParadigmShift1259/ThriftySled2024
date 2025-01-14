#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"

class GoToPositionCommand: public frc2::CommandHelper<frc2::Command, GoToPositionCommand>
{
    public:
        explicit GoToPositionCommand(ISubsystemAccess& subsystemAccess, bool bIsBlue);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        DriveSubsystem&        m_driveSubsystem;
        VisionSubsystem&        m_visionSubsystem;
        // LEDSubsystem&           m_led;
        double m_targetX;
        double m_targetY;
        double m_targetRot;
        bool m_bIsBlue = false;

        frc::Timer m_timer;

        // LEDSubsystem::Color c_colorWhite = LEDSubsystem::CreateColor(255, 255, 255, 10);

		wpi::log::BooleanLogEntry m_logStartGoToPositionCommand;
		wpi::log::BooleanLogEntry m_logGoToPositionCommandFlipped;
};
