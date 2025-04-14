#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "ISubsystemAccess.h"

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

using namespace pathplanner;

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

class GoToPositionCommand : public frc2::CommandHelper<frc2::Command, GoToPositionCommand>
{
    public:

        explicit GoToPositionCommand(ISubsystemAccess& subsystemAccess, EMoveDirection elmr, std::shared_ptr<PathPlannerPath>& path);
        //explicit GoToPositionCommand(ISubsystemAccess& subsystemAccess, EMoveDirection elmr);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        // PathPlannerPath m_path;
        PathConstraints m_pathConstraints { 1.0_mps, 1.0_mps_sq, 90_deg_per_s, 180_deg_per_s_sq };

        DriveSubsystem&        m_driveSubsystem;
        VisionSubsystem&        m_visionSubsystem;
#ifdef LED
        LEDSubsystem&           m_led;
#endif
        double m_targetX;
        double m_targetY;
        double m_targetRot;
        EMoveDirection m_elmr;
        bool m_bJogging;
        //std::optional<frc2::CommandPtr>& m_pathCmd;
        std::shared_ptr<PathPlannerPath> m_path;

        frc::Timer m_timer;

        wpi::log::BooleanLogEntry m_logStartGoToPositionCommand;
	wpi::log::BooleanLogEntry m_logGoToPositionCommandFlipped;
};
