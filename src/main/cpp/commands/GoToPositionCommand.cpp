
#include "commands/GoToPositionCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

const units::length::meter_t c_halfRobotSize = 14.5_in + 3.0_in;  // Half robot width/length + bumper 29 / 2 = 14.5 + 3.0]
const units::length::meter_t c_reefPoleOffset = 6.5_in;  // reef poles are 13 inches apart, this is half
const double c_cosine30 = sqrt(3) * 0.5;
#ifdef TAG_3
const units::length::meter_t c_targetRedX = 455.15_in;//530.49_in;
const units::length::meter_t c_targetRedY = 317.15_in;//130.17_in;
const units::angle::degree_t c_targetRedRot = 90_deg;//120_deg;
#else
const units::length::meter_t c_targetRedX = 530.49_in;//530.49_in;
const units::length::meter_t c_targetRedY = 130.17_in;
const units::angle::degree_t c_targetRedRot = 120_deg;
#endif
const double c_targetPodiumX = (2.896_m - c_halfRobotSize).value();
const double c_targetPodiumY = 4.106;
const double c_tolerance = 0.02;
const double c_minInput = 0.07;

constexpr int c_tagIdReefBlue = 17;
const double c_targetSpeakerBlueX = 1.23;
const double c_targetSpeakerBlueY = 5.35;

const double c_targetSpeakerRedX = c_targetSpeakerBlueX;
const double c_targetSpeakerRedY = 4.106 - 1.448;

const double c_targetReefBlueX = (1.933_m - 0.050_m).value();  // 5cm bias on shooter/intake
const double c_targetReefBlueY = (8.111_m - c_halfRobotSize).value();
const double c_targetReefBlueRot = -90.0;

#ifdef TAG_3
const int c_tagIdReefRed = 3;
const double c_targetReefRedX = (c_targetRedX).value(); 
const double c_targetReefRedY = (c_targetRedY - c_halfRobotSize).value(); 
const double c_targetReefRedRot = c_targetRedRot.value();
#else
const int c_tagIdReefRed = 6;
const double c_targetReefRedX = (c_targetRedX + c_halfRobotSize * 0.5).value(); //0.5 is sin(30)
const double c_targetReefRedY = (c_targetRedY - c_halfRobotSize * sqrt(3) * 0.5).value(); //root(3)/2 is cos(30)
const double c_targetReefRedRot = c_targetRedRot.value();
const double c_targetReefRedOffsetX = c_reefPoleOffset.value() * c_cosine30;
const double c_targetReefRedOffsetY = c_reefPoleOffset.value() * 0.5;
const double c_targetLeftReefRedX = c_targetReefRedX - c_targetReefRedOffsetX;
const double c_targetLeftReefRedY = c_targetReefRedY - c_targetReefRedOffsetX;
const double c_targetRightReefRedX = c_targetReefRedX + c_targetReefRedOffsetX;
const double c_targetRightReefRedY = c_targetReefRedY + c_targetReefRedOffsetX;
#endif


const units::velocity::meters_per_second_t c_defaultGoToReefMaxSpeed = 2.0_mps;

GoToPositionCommand::GoToPositionCommand(ISubsystemAccess& subsystemAccess, ELeftMiddleRight elmr)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
    // , m_led(subsystemAccess.GetLED())
    , m_targetX(0)
    , m_targetY(0)
    , m_targetRot(0)
    , m_elmr(elmr)
{
    // AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision(), &subsystemAccess.GetLED()});
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartGoToPositionCommand = wpi::log::BooleanLogEntry(log, "/GoToPositionCommand/startCommand");
    m_logGoToPositionCommandFlipped = wpi::log::BooleanLogEntry(log, "/GoToPositionCommand/startCommand");

    //frc::SmartDashboard::PutNumber("GoReefMaxSpd", c_defaultGoToReefMaxSpeed.value());
    frc::SmartDashboard::PutNumber("GoReefMaxAnglSpd", 90.0);
    frc::SmartDashboard::PutNumber("targetX", m_targetX);
    frc::SmartDashboard::PutNumber("targetY", m_targetY);
    frc::SmartDashboard::PutNumber("targetRot", m_targetRot);
    frc::SmartDashboard::PutNumber("gotoMaxX", 3.0);
    frc::SmartDashboard::PutNumber("gotoMaxY", 3.0);
}

void GoToPositionCommand::Initialize()
{
    // if (m_led.GetCurrentAction() == LEDSubsystem::CurrentAction::kReefPosition)
    // {
    //     m_led.SetCurrentAction(LEDSubsystem::CurrentAction::kReefMovement);
    // }
    // m_led.SetAnimation(c_colorWhite, LEDSubsystem::Animation::kFlow);
    m_timer.Reset();
    m_timer.Start();
    // m_visionSubsystem.EnableReefLEDs();

    m_targetX = frc::SmartDashboard::GetNumber("targetX", m_targetX);
    m_targetY = frc::SmartDashboard::GetNumber("targetY", m_targetY);
    m_targetRot = frc::SmartDashboard::GetNumber("targetRot", m_targetRot);
}

void GoToPositionCommand::Execute()
{
    //const double c_maxX = 3.0;
    //const double c_maxY = 3.0;
    double c_maxX = frc::SmartDashboard::GetNumber("gotoMaxX", 3.0);
    double c_maxY = frc::SmartDashboard::GetNumber("gotoMaxY", 3.0);
    
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
        // int tagId = m_visionSubsystem.GetTagId();
        m_targetX = c_targetReefRedX;
        m_targetY = c_targetReefRedY;
        if (m_elmr == eLeft)
        {
            m_targetX = c_targetLeftReefRedX;
            m_targetY = c_targetLeftReefRedY;
        } 
        else if (m_elmr == eRight)
        {
            m_targetX = c_targetRightReefRedX;
            m_targetY = c_targetRightReefRedY;
        }
        m_targetRot  = c_targetReefRedRot;
        

        if (xDiff >= c_tolerance && xDiff < c_maxX)
        {
            yInput = (m_targetX - x) / c_maxX;

            if (fabs(yInput) < c_minInput)
            {
                if (yInput < 0.0)
                {
                    yInput = -c_minInput;
                }
                else
                {
                    yInput = c_minInput;
                }
            }
        }

        if (yDiff >= c_tolerance && yDiff < c_maxY)
        {
            xInput = (y - m_targetY) / c_maxY;
            
            if (fabs(xInput) < c_minInput)
            {
                if (xInput < 0.0)
                {
                    xInput = -c_minInput;
                }
                else
                {
                    xInput = c_minInput;
                }
            }
        }

        if (rotDiff >= c_tolerance && rotDiff < c_maxRot)
        {
            rotInput = (rotation - m_targetRot) / c_maxRot;
        }

        //units::velocity::meters_per_second_t maxSpeed = units::velocity::meters_per_second_t{frc::SmartDashboard::GetNumber("GoReefMaxSpd", c_defaultGoToReefMaxSpeed.value())};
        units::velocity::meters_per_second_t maxSpeed = c_defaultGoToReefMaxSpeed;
        xSpeed = xInput * maxSpeed; 
        ySpeed = yInput * maxSpeed; 
        units::angular_velocity::degrees_per_second_t maxAngularSpeed = units::angular_velocity::degrees_per_second_t{frc::SmartDashboard::GetNumber("GoReefMaxAnglSpd", 90.0)};
        rotSpeed = rotInput * maxAngularSpeed;         
        
        m_driveSubsystem.Drive(xSpeed, ySpeed, rotSpeed, false);
    }

    // printf("tv %s x %.3f y %.3f rot %.3f xDiff %.3f yDiff %.3f rotDiff %.3f xInput %.3f yInput %.3f rotInput %.3f xSpeed %.3f yspeed %.3f rotSpeed %.3f\n"
    //     , m_visionSubsystem.IsValidReef() ? "true" : "false"
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
    // m_visionSubsystem.DisableReefLEDs();
}
