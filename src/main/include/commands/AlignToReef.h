#include "Constants.h"
#include "ISubsystemAccess.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

//#include <pathplanner/lib/path/PathPlannerPath.h>
//#include <pathplanner/lib/auto/AutoBuilder.h>
//#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

//using namespace pathplanner;

// Adapted from Spartronics 4915
// https://github.com/Spartronics4915/2025-Reefscape
// https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit)
class AlignToReef : public frc2::CommandHelper<frc2::Command, AlignToReef>
{  
    explicit AlignToReef(ISubsystemAccess& subsystemAccess, Pose2d goalPose);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

  private:
    DriveSubsystem&             m_driveSubsystem;
#ifdef LED
    LEDSubsystem&               m_ledSubsystem;
#endif
//    PathPlannerTrajectoryState  m_goalState;
    // PPHolonomicDriveController  m_driveController{ PIDConstants(DriveConstants::c_HolomonicTranslateP, 0.0, 0.0), 
    //                                                PIDConstants(DriveConstants::c_HolomonicRotateP, 0.0, 0.0) };

    wpi::log::BooleanLogEntry   m_logStartCommand;

    // public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    // public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
    // public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();

    // public AlignToReef(SwerveSubsystem mSwerve, AprilTagFieldLayout field) {
    //     this.mSwerve = mSwerve;

    //     Arrays.stream(AprilTagRegion.kReef.blue()).forEach((i) -> {
    //         field.getTagPose(i).ifPresent((p) -> {
    //             blueReefTagPoses.add(new Pose2d(
    //                 p.getMeasureX(),
    //                 p.getMeasureY(),
    //                 p.getRotation().toRotation2d()
    //             ));
    //         });
    //     });

    //     Arrays.stream(AprilTagRegion.kReef.red()).forEach((i) -> {
    //         field.getTagPose(i).ifPresent((p) -> {
    //             redReefTagPoses.add(new Pose2d(
    //                 p.getMeasureX(),
    //                 p.getMeasureY(),
    //                 p.getRotation().toRotation2d()
    //             ));
    //         });
    //     });

    //     Arrays.stream(AprilTagRegion.kReef.both()).forEach((i) -> {
    //         field.getTagPose(i).ifPresent((p) -> {
    //             allReefTagPoses.add(new Pose2d(
    //                 p.getMeasureX(),
    //                 p.getMeasureY(),
    //                 p.getRotation().toRotation2d()
    //             ));
    //         });
    //     });
    // }

    // public Command generateCommand(BranchSide side) {
    //     return Commands.defer(() -> {
    //         var branch = getClosestBranch(side, mSwerve);
    //         desiredBranchPublisher.accept(branch);
    
    //         return getPathFromWaypoint(getWaypointFromBranch(branch));
    //     }, Set.of());
    // }


    // public Command generateCommand(final ReefSide reefTag, BranchSide side) {
    //     return Commands.defer(() -> {
    //         var branch = getBranchFromTag(reefTag.getCurrent(), side);
    //         desiredBranchPublisher.accept(branch);
    
    //         return getPathFromWaypoint(getWaypointFromBranch(branch));
    //     }, Set.of());
    // }

    // private Command getPathFromWaypoint(Pose2d waypoint) {
    //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //         new Pose2d(mSwerve.getPose().getTranslation(), getPathVelocityHeading(mSwerve.getFieldVelocity(), waypoint)),
    //         waypoint
    //     );

    //     if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
    //         return 
    //         Commands.sequence(
    //             Commands.print("start position PID loop"),
    //             PositionPIDCommand.generateCommand(mSwerve, waypoint, kAlignmentAdjustmentTimeout),
    //             Commands.print("end position PID loop")
    //         );
    //     }

    //     PathPlannerPath path = new PathPlannerPath(
    //         waypoints, 
    //         kPathConstraints,
    //         new IdealStartingState(getVelocityMagnitude(mSwerve.getFieldVelocity()), mSwerve.getHeading()), 
    //         new GoalEndState(0.0, waypoint.getRotation())
    //     );

    //     path.preventFlipping = true;

    //     return AutoBuilder.followPath(path).andThen(
    //         Commands.print("start position PID loop"),
    //         PositionPIDCommand.generateCommand(mSwerve, waypoint, kAlignmentAdjustmentTimeout),
    //         Commands.print("end position PID loop")
    //     );
    // }

    //  * @param cs field relative chassis speeds
    // private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
    //     if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
    //         var diff = target.minus(mSwerve.getPose()).getTranslation();
    //         return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
    //     }
    //     return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    // }
};
