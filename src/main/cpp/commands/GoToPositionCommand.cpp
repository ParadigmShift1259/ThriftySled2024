
#include "commands/GoToPositionCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

const units::length::meter_t c_halfRobotSize = 14.5_in + 3.0_in;  // Half robot width/length + bumper 29 / 2 = 14.5 + 3.0
const units::length::meter_t c_targetRedX = 530.49_in;
const units::length::meter_t c_targetRedY = 130.17_in;
const units::angle::degree_t c_targetRedRot = 120_deg;

const double c_targetPodiumX = (2.896_m - c_halfRobotSize).value();
const double c_targetPodiumY = 4.106;
const double c_tolerance = 0.02;
const double c_minInput = 0.07;

constexpr int c_tagIdAmpBlue = 17;
const double c_targetSpeakerBlueX = 1.23;
const double c_targetSpeakerBlueY = 5.35;

const double c_targetSpeakerRedX = c_targetSpeakerBlueX;
const double c_targetSpeakerRedY = 4.106 - 1.448;

const double c_targetAmpBlueX = (1.933_m - 0.050_m).value();  // 5cm bias on shooter/intake
const double c_targetAmpBlueY = (8.111_m - c_halfRobotSize).value();
const double c_targetAmpBlueRot = -90.0;

const int c_tagIdAmpRed = 6;
const double c_targetAmpRedX = (c_targetRedX + c_halfRobotSize * 0.5).value(); //0.5 is sin(30)
const double c_targetAmpRedY = (c_targetRedY - c_halfRobotSize * sqrt(3) * 0.5).value(); //root(3)/2 is cos(30)
const double c_targetAmpRedRot = c_targetRedRot.value();

const units::velocity::meters_per_second_t c_defaultGoToAmpMaxSpeed = 4.5_mps;

GoToPositionCommand::GoToPositionCommand(ISubsystemAccess& subsystemAccess, bool bIsBlue)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
    // , m_led(subsystemAccess.GetLED())
    , m_targetX(bIsBlue ? c_targetAmpBlueX : c_targetAmpRedX)
    , m_targetY(bIsBlue ? c_targetAmpBlueY : c_targetAmpRedY)
    , m_targetRot(bIsBlue ? c_targetAmpBlueRot : c_targetAmpRedRot)
    , m_bIsBlue(bIsBlue)
{
    // AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision(), &subsystemAccess.GetLED()});
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartGoToPositionCommand = wpi::log::BooleanLogEntry(log, "/GoToPositionCommand/startCommand");
    m_logGoToPositionCommandFlipped = wpi::log::BooleanLogEntry(log, "/GoToPositionCommand/startCommand");

    //frc::SmartDashboard::PutNumber("GoAmpMaxSpd", c_defaultGoToAmpMaxSpeed.value());
    frc::SmartDashboard::PutNumber("GoAmpMaxAnglSpd", 360.0);
    frc::SmartDashboard::PutNumber("targetX", m_targetX);
    frc::SmartDashboard::PutNumber("targetY", m_targetY);
    frc::SmartDashboard::PutNumber("targetRot", m_targetRot);
}

void GoToPositionCommand::Initialize()
{
    // if (m_led.GetCurrentAction() == LEDSubsystem::CurrentAction::kAmpPosition)
    // {
    //     m_led.SetCurrentAction(LEDSubsystem::CurrentAction::kAmpMovement);
    // }
    // m_led.SetAnimation(c_colorWhite, LEDSubsystem::Animation::kFlow);
    m_timer.Reset();
    m_timer.Start();
    // m_visionSubsystem.EnableAmpLEDs();

    m_targetX = frc::SmartDashboard::GetNumber("targetX", m_targetX);
    m_targetY = frc::SmartDashboard::GetNumber("targetY", m_targetY);
    m_targetRot = frc::SmartDashboard::GetNumber("targetRot", m_targetRot);
}

void GoToPositionCommand::Execute()
{
    const double c_maxX = 3.0;
    const double c_maxY = 3.0;
    const double c_maxRot = 45.0;
    auto x = m_visionSubsystem.GetX();
    auto y = m_visionSubsystem.GetY();
    auto rotation = m_driveSubsystem.GetGyroAzimuthDeg().value();
    auto xInput = 0.0;
    auto yInput = 0.0;
    auto rotInput = 0.0;
    auto xDiff = fabs(m_targetX - x);
    auto yDiff = fabs(m_targetY - y);
    auto rotDiff = fabs(m_targetRot - rotation);
    auto xSpeed = 0.0_mps;
    auto ySpeed = 0.0_mps;
    auto rotSpeed = 0.0_deg_per_s;
    
    if (m_visionSubsystem.IsValidReef())
    {
        int tagId = m_visionSubsystem.GetTagId();
        bool bBlueAllianceFromTagId = (tagId == c_tagIdAmpBlue);
        if (bBlueAllianceFromTagId != m_bIsBlue)
        {
            m_bIsBlue = bBlueAllianceFromTagId;
            m_targetX = (m_bIsBlue ? c_targetAmpBlueX : c_targetAmpRedX);
            m_targetY = (m_bIsBlue ? c_targetAmpBlueY : c_targetAmpRedY);
            m_targetRot = (m_bIsBlue ? c_targetAmpBlueRot : c_targetAmpRedRot);
            m_logGoToPositionCommandFlipped.Append(true);
        }
        else
        {
            m_logGoToPositionCommandFlipped.Append(false);
        }

        if (xDiff >= c_tolerance && xDiff < c_maxX)
        {
            if (tagId == c_tagIdAmpRed)
            {
                yInput = (m_targetX - x) / c_maxX;
            }
            else
            {
                yInput = (x - m_targetX) / c_maxX;
            }
            if (yInput < 0.0)
            {
                yInput = std::min(-c_minInput, yInput);
            }
            else
            {
                yInput = std::max(c_minInput, yInput);
            }
        }

        if (yDiff >= c_tolerance && yDiff < c_maxY)
        {
            if (tagId == c_tagIdAmpRed)
            {
                xInput = (y - m_targetY) / c_maxY;
            }
            else
            {
                xInput = (m_targetY - y) / c_maxY;
            }
            if (xInput < 0.0)
            {
                xInput = std::min(-c_minInput, xInput);
            }
            else
            {
                xInput = std::max(c_minInput, xInput);
            }
        }

        if (rotDiff >= c_tolerance && rotDiff < c_maxRot)
        {
            rotInput = (rotation - m_targetRot) / c_maxRot;
        }

        //units::velocity::meters_per_second_t maxSpeed = units::velocity::meters_per_second_t{frc::SmartDashboard::GetNumber("GoAmpMaxSpd", c_defaultGoToAmpMaxSpeed.value())};
        units::velocity::meters_per_second_t maxSpeed = c_defaultGoToAmpMaxSpeed;
        xSpeed = xInput * maxSpeed; 
        ySpeed = yInput * maxSpeed; 
        units::angular_velocity::degrees_per_second_t maxAngularSpeed = units::angular_velocity::degrees_per_second_t{frc::SmartDashboard::GetNumber("GoAmpMaxAnglSpd", 360.0)};
        // units::angular_velocity::degrees_per_second_t maxAngularSpeed = units::angular_velocity::degrees_per_second_t{120.0};
        rotSpeed = rotInput * maxAngularSpeed;         
        
        m_driveSubsystem.Drive(xSpeed, ySpeed, rotSpeed, false);
    }

    // printf("tv %s x %.3f y %.3f rot %.3f xDiff %.3f yDiff %.3f rotDiff %.3f xInput %.3f yInput %.3f rotInput %.3f xSpeed %.3f yspeed %.3f rotSpeed %.3f\n"
    //     , m_visionSubsystem.IsValidAmp() ? "true" : "false"
    //     , x
    //     , y
    //     , rotation
    //     , xDiff
    //     , yDiff
    //     , rotDiff
    //     , xInput
    //     , yInput
    //     , rotInput
    //     , xSpeed.value()
    //     , ySpeed.value()
    //     , rotSpeed.value());
}

bool GoToPositionCommand::IsFinished()
{
    auto x = m_visionSubsystem.GetX();
    auto y = m_visionSubsystem.GetY();

    bool finished = fabs(m_targetY - y) < c_tolerance && fabs(m_targetX - x) < c_tolerance;

    if (!finished) 
    {
        auto xDiff = fabs(m_targetX - x);
        auto yDiff = fabs(m_targetY - y);
        printf("tv %s x %.3f y %.3f xDiff %.3f yDiff %.3f \n"
        , m_visionSubsystem.IsValidReef() ? "true" : "false"
        , x
        , y
        , xDiff
        , yDiff
        );
    }

    return finished;
}

void GoToPositionCommand::End(bool interrupted)
{
    // m_led.SetAnimation(c_colorWhite, LEDSubsystem::kStrobe);
    m_driveSubsystem.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
    m_visionSubsystem.SetPositionStarted(false);
    // m_visionSubsystem.DisableAmpLEDs();
}
