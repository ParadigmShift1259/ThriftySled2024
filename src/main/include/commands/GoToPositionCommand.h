#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"

enum EMoveDirection
{
    eLeft
    , eMiddle
    , eRight
    
    , eJogLeft
    , eJogRight
    , eJogForward
    , eJogBackward
} ;

class GoToPositionCommand: public frc2::CommandHelper<frc2::Command, GoToPositionCommand>
{
    public:
        explicit GoToPositionCommand(ISubsystemAccess& subsystemAccess, EMoveDirection elmr);
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
        EMoveDirection m_elmr;
        bool m_bJogging;

        frc::Timer m_timer;

        // LEDSubsystem::Color c_colorWhite = LEDSubsystem::CreateColor(255, 255, 255, 10);

		wpi::log::BooleanLogEntry m_logStartGoToPositionCommand;
		wpi::log::BooleanLogEntry m_logGoToPositionCommandFlipped;
};
