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

#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace rev::spark;

// constexpr double c_defaultResetTurns = 0.0;
// constexpr double c_defaultParkTurns = -30.0;
// constexpr double c_defaultHighTurns = -150.0;

class ClimberSubsystem : public frc2::SubsystemBase
{
public:

    ClimberSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the climber at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Set(double speed);

    void Stop() { m_motor.StopMotor(); }

    void GoToPosition(double position);

    void GoToPositionRel(double position);


    // enum Position {
    //     kDefaultPosition,
    //     kResetPosition = kDefaultPosition,
    //     kHighPosition,
    //     kParkPosition
    // };

private:
    SparkMax m_motor;
    SparkRelativeEncoder m_relativeEnc = m_motor.GetEncoder();    
    SparkClosedLoopController m_closedLoopController = m_motor.GetClosedLoopController();

    double m_climbPosition = 1.0;

    wpi::log::DoubleLogEntry m_log;

    double m_direction = 1.0;
};
