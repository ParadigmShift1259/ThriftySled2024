#pragma once

#include <wpi/DataLog.h>

#include <ConstantsCANIDs.h>

#include <ctre/phoenix/led/CANdle.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <ctre/phoenix/ErrorCode.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>
#include <frc/AddressableLED.h>

using namespace ctre::phoenix::led;

class LEDSubsystem : public frc2::SubsystemBase
{
  public:
    LEDSubsystem();
    void Periodic() override;
    struct Color
    {
      int red = 0;
      int green = 0;
      int blue = 0;
      int white = 0;
    };

    enum Animation
    {
      kDefaultAnim,
      kSolid = kDefaultAnim,
      kFade,
      kFlow,
      kStrobe,
      kBlank
    };

    enum CurrentAction
    {
      kDefaultAction,
      kIdle = kDefaultAction,
      kIntaking,
      kPreShoot,
      kShootMovement,
      kShoot,
      kAmpPosition,
      kAmpMovement,
      kAmpShoot,
      kClimbing,
      kClimbFinish
    };

    void SetAnimation(Color rgbw, Animation animate);
    static Color CreateColor(int r, int g, int b, int w);

    bool IsRobotBusy() { return m_currentAction != kIdle; }
    // void SetRobotBusy(bool value) { m_busy = value; }

    void SetDefaultColor(Color color) { m_defaultColor = color; }
    Color GetDefaultColor() { return m_defaultColor; }

    void SetCurrentAction(CurrentAction action) { m_currentAction = action; }
    CurrentAction GetCurrentAction() { return m_currentAction; }
    
  private:
#ifndef THING1
    CANdleConfiguration m_candleConfig;
    wpi::log::DoubleLogEntry m_log;
    CANdle m_candle{kLEDCANID};
#endif
    static constexpr double c_defaultSpeed = 0.5;

    static constexpr int c_ledNum = 16;
    static constexpr int c_ledOffset = 8;

    ColorFlowAnimation m_colorFlowAnimation{0, 0, 0, 0, c_defaultSpeed, c_ledNum, ColorFlowAnimation::Forward, c_ledOffset};
    SingleFadeAnimation m_singleFadeAnimation{0, 0, 0, 0, c_defaultSpeed, c_ledNum, c_ledOffset};
    StrobeAnimation m_strobeAnimation{0, 0, 0, 0, c_defaultSpeed, c_ledNum, c_ledOffset};

    Color m_defaultColor;
    CurrentAction m_currentAction;

    double m_speed;
    bool m_busy;
};
