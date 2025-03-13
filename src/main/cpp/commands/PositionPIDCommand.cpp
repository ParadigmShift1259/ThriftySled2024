#include "commands/PositionPIDCommand.h"

#include <frc/MathUtil.h>

constexpr Rotation2d                           c_RotationTolerance{ 2.0_deg };
constexpr units::length::meter_t               c_PositionTolerance{ 0.4_in };
constexpr units::velocity::meters_per_second_t c_SpeedTolerance{ 0.021_fps };   // Quarter inch per second

PositionPIDCommand::PositionPIDCommand(ISubsystemAccess& subsystemAccess, Pose2d goalPose)
    : m_driveSubsystem(subsystemAccess.GetDrive())
{
    m_goalState.pose = goalPose;

    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/PositionPIDCommand/startCommand");
}

void PositionPIDCommand::Initialize()
{
    printf("PositionPIDCommand::Initialize\n");
    m_logStartCommand.Append(true);
    m_timer.Reset();
    m_timer.Start();
}

void PositionPIDCommand::Execute()
{
    auto chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(m_driveSubsystem.GetPose(), m_goalState);
    m_driveSubsystem.Drive(chassisSpeeds, DriveFeedforwards{});
}

bool PositionPIDCommand::IsFinished()
{
    Pose2d diff = m_driveSubsystem.GetPose().RelativeTo(m_goalState.pose);

    constexpr units::angle::turn_t zeroTurns{0.0};
    constexpr units::angle::turn_t oneTurn{1.0}; 
    units::angle::turn_t turnDiff{diff.Rotation().Radians()};          // isNear expects rotations (turn count)
    units::angle::turn_t turnTolerance{c_RotationTolerance.Radians()}; 

    auto rotationOk = frc::IsNear(
          zeroTurns      // expected
        , turnDiff       // actual
        , turnTolerance  // tolerance
        , zeroTurns      // min
        , oneTurn        // max
    );

    printf("PositionPIDCommand turnDiff %.3f translation norm %.3f speed %.3f\n", turnDiff.value(), diff.Translation().Norm().value(), m_driveSubsystem.GetSpeed().value());

    auto positionOk = diff.Translation().Norm() < c_PositionTolerance;

    auto speedOk = m_driveSubsystem.GetSpeed() < c_SpeedTolerance;
//#define BOOL_STR(b) b ? "true" : "false"
 //   printf("PositionPIDCommand end conditions Rot: %s Pos: %s Spd: %s\n", BOOL_STR(rotationOk), BOOL_STR(positionOk), BOOL_STR(speedOk));
    
//    return (rotationOk && positionOk && speedOk) || m_timer.HasElapsed(0.150_s);    // 0.075 sec => 3 or 4 20ms periodic cycles
    return (rotationOk && positionOk && speedOk) || m_timer.HasElapsed(0.075_s);    // 0.075 sec => 3 or 4 20ms periodic cycles
    //     endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
}

void PositionPIDCommand::End(bool interrupted)
{
    printf("PositionPIDCommand::End\n");
    m_driveSubsystem.Stop();
    m_logStartCommand.Append(false);
}

