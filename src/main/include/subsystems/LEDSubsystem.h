#pragma once

#include <wpi/DataLog.h>

#include <ConstantsCANIDs.h>

#include <ctre/phoenix/led/CANdle.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <ctre/phoenix/ErrorCode.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>

using namespace ctre::phoenix::led;

struct RBGAColor
{
  int red = 0;
  int green = 0;
  int blue = 0;
  int white = 0;
};

//                                                 red green blue white
const RBGAColor c_colorPink   = {  80,  10,  15,   0 };
const RBGAColor c_colorGreen  = {  13,  80,   0,   0 };
const RBGAColor c_colorBlue   = {   0,   0, 255,   0 };
const RBGAColor c_colorRed    = { 255,   0,   0,   0 };
const RBGAColor c_colorOrange = {  43,   6,   0, 255 };
const RBGAColor c_colorYellow = { 255,   0,   0,   0 };
const RBGAColor c_colorPurple = {  80,   0,  80,   0 };
const RBGAColor c_colorBlack  = {   0,   0,   0,   0 };
const RBGAColor c_colorWhite  = { 255, 255, 255,  10 };

const RBGAColor c_defaultColor = c_colorWhite;

class LEDSubsystem : public frc2::SubsystemBase
{
  public:
    LEDSubsystem();
    void Periodic() override;

    enum EAnimation
    {
      kDefaultAnim,
      kSolid = kDefaultAnim,
      kFade,
      kFlow,
      kStrobe,
      kScanner,
      kBlank
    };
    
    enum ECurrentAction
    {
      kDefaultAction,
      kIdle = kDefaultAction,
      kIntaking,
      kHasCoral,
      kTagVisible,
      kPreCoral,
      kPostCoral,
      kElevator,
      kClimbing,
      kSequenceStart,
      kFollowPath,
      kClimbFinish
    };

    void SetAnimation(RBGAColor rgbw, EAnimation animate);

    bool IsRobotBusy() { return m_currentAction != kIdle; }

    //void SetDefaultColor(RBGAColor color) { m_defaultColor = color; }
    RBGAColor GetDefaultColor() { return m_defaultColor; }

    void SetCurrentAction(ECurrentAction action) { m_currentAction = action; }

    ECurrentAction GetCurrentAction() { return m_currentAction; }
    
  private:
    // Convienince function to set LED color by accessing color struct components
    void SetColor(const RBGAColor& color) 
    { 
      m_currentColor = color;
      m_candle.SetLEDs(color.red, color.green, color.blue, color.white, c_ledOffset, c_ledNum);
    }
    // Convienince function to set an animation color by accessing color struct components
    void SetColor(BaseTwoSizeAnimation& animation, const RBGAColor& color)
    {
      m_currentColor = color;
      animation.SetR(color.red);
      animation.SetG(color.green);
      animation.SetB(color.blue);
      animation.SetW(color.white);
    }

    bool IsSameColor(const RBGAColor& color1, const RBGAColor& color2)
    {
      return    color1.red == color2.red 
             && color1.green == color2.green
             && color1.blue == color2.blue
             && color1.white == color2.white;
    }

    CANdleConfiguration m_candleConfig;
    wpi::log::DoubleLogEntry m_log;
    CANdle m_candle{kLEDCANID};
    std::optional<frc::DriverStation::Alliance> m_alliance;

    static constexpr double c_defaultSpeed = 0.5;
    static constexpr int c_ledNum = 16;
    static constexpr int c_ledOffset = 8;

    // Animation c'tor args
    // First 4 are red, green, blue, white (brightness) [0 to 255]
    // speed	How fast should the color travel the strip [0, 1]
    // numLed	How many LEDs the CANdle controls
    // ledOffset	Where to start the animation
    //
    // Animation that gradually lights the entire LED strip one LED at a time
    // Additional args
    // direction	Forward, Backward 
    ColorFlowAnimation m_colorFlowAnimation{0, 0, 0, 0, c_defaultSpeed, c_ledNum, ColorFlowAnimation::Forward, c_ledOffset};
    // Animation that fades into and out of a specified color.
    SingleFadeAnimation m_singleFadeAnimation{0, 0, 0, 0, c_defaultSpeed, c_ledNum, c_ledOffset};
    // Animation that strobes the LEDs a specified color
    StrobeAnimation m_strobeAnimation{0, 0, 0, 0, c_defaultSpeed, c_ledNum, c_ledOffset};
    // Animation that sends a pocket of light across the LED strip (see https://en.wikipedia.org/wiki/Knight_Rider)
    // Additional args
    // BounceMode mode  Front, Center, Back; default Front
    // int size         Size of the group of lit LEDs [0 to 7]; defalt 2 
    // int ledOffset    Where to start the animation; default 0
    LarsonAnimation m_larsonAnimation{0, 0, 0, 0, c_defaultSpeed, c_ledNum, LarsonAnimation::Front, 3, c_ledOffset};

    RBGAColor       m_defaultColor;
    RBGAColor       m_currentColor = c_colorBlack;
    ECurrentAction  m_currentAction;
};


