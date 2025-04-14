#include "Constants.h"
#include "ISubsystemAccess.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

using namespace pathplanner;

// Adapted from Spartronics 4915
// https://github.com/Spartronics4915/2025-Reefscape
// https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit)
class PositionPIDCommand : public frc2::CommandHelper<frc2::Command, PositionPIDCommand>
{
  public:
    explicit PositionPIDCommand(ISubsystemAccess& subsystemAccess, Pose2d goalPose);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

  private:
    DriveSubsystem&             m_driveSubsystem;
    PathPlannerTrajectoryState  m_goalState;
    PPHolonomicDriveController  m_driveController{ PIDConstants(DriveConstants::c_HolomonicTranslateP, 0.0, 0.0), 
                                                   PIDConstants(DriveConstants::c_HolomonicRotateP, 0.0, 0.0) };
    frc::Timer m_timer;

    wpi::log::BooleanLogEntry   m_logStartCommand;
};
