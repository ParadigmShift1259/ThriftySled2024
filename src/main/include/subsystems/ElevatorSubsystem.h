#pragma once

#include <wpi/DataLog.h>

#include "Constants.h"
#include "ConstantsDigitalInputs.h"
#include "ConstantsCANIDs.h"

#include <frc/DigitalInput.h>

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>

#include <wpi/deprecated.h>
WPI_IGNORE_DEPRECATED
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
WPI_UNIGNORE_DEPRECATED

#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace rev::spark;

//All temporary values 1/18/25
constexpr double c_defaultResetTurns = 0.0;
constexpr double c_defaultParkTurns = 0.0;
constexpr double c_defaultL1Turns = 0.0;
constexpr double c_defaultL2Turns =  2.5; // originally 2.0
constexpr double c_defaultL3Turns = 15.5; 
constexpr double c_defaultL4Turns = 38.0;
constexpr double c_defaultLoadTurns = 10.0;
constexpr double c_algaeRemovalL3_4 = 28.0;
constexpr double c_algaeRemovalL2_3 = 14.0;

class ElevatorSubsystem : public frc2::SubsystemBase
{
public:
    ElevatorSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    void Stop() { m_followMotor.StopMotor(); m_leadMotor.StopMotor(); }

    void GoToPosition(double position);

    void ElevatorReset(){ m_leadRelativeEnc.SetPosition(0.0); m_followRelativeEnc.SetPosition(0.0);}

    double GetCurrentPosition(){return m_leadRelativeEnc.GetPosition();}
    void GotoPositionRel(double relPos);

    bool IsAtPosition(double level);

    enum Position {
        kDefaultPosition,
        kResetPosition = kDefaultPosition,
        kHighPosition,
        kParkPosition
    };

private:
    SparkBaseConfig m_leadConfig{};
    SparkBaseConfig m_followConfig{};
    SparkFlex m_leadMotor;
    SparkRelativeEncoder m_leadRelativeEnc = m_leadMotor.GetEncoder();    
    SparkClosedLoopController m_leadPIDController = m_leadMotor.GetClosedLoopController();
    SparkFlex m_followMotor;
    SparkRelativeEncoder m_followRelativeEnc = m_followMotor.GetEncoder();    
    SparkClosedLoopController m_followPIDController = m_followMotor.GetClosedLoopController();

    wpi::log::DoubleLogEntry m_log;

    double m_leadDirection = 1.0;
    double m_followDirection = 1.0;

    double m_position = 0.0;
    ClosedLoopSlot m_slot = ClosedLoopSlot::kSlot0;
};
