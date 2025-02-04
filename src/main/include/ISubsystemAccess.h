#pragma once

#ifndef __SUBSYSTEMACCESS_H__
#define __SUBSYSTEMACCESS_H__

#include "Constants.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/CoralManipulatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#ifdef LED
#include "subsystems/LEDSubsystem.h"
#endif

// #include "subsystems/ClimberSubsystem.h"

#include <frc/DataLogManager.h>

class ISubsystemAccess
{
public:
    virtual DriveSubsystem&        GetDrive() = 0;
    virtual VisionSubsystem&       GetVision() = 0;
    virtual ElevatorSubsystem&       GetElevator() = 0;
    virtual CoralManipulatorSubsystem& GetCoral() = 0; 
#ifdef LED
    virtual LEDSubsystem&          GetLED() = 0;
#endif
    virtual IntakeSubsystem&       GetIntake() = 0;
    // virtual ShooterSubsystem&      GetShooter() = 0;
    // virtual ClimberSubsystem&      GetClimber() = 0;

    virtual wpi::log::DataLog&     GetLogger() = 0;
};

#endif  //ndef __SUBSYSTEMACCESS_H__
