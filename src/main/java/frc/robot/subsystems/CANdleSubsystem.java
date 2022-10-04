// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.*;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSubsystem extends SubsystemBase {

  private CANdle m_candle;
  private final int LedCount = 8;
  private Animation m_toAnimate = null;


  private CANdle candle;
  // private final int LedCount = 8;
  // private Animation m_toAnimate = null;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
}
/** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {

    if (Constants.RobotProperties.isCANdle) {
      // m_candle = new CANdle(Constants.CANdleConstants.CANdlePort, "FastFD");

      System.out.println("Initializing CANdle");

      candle = new CANdle(Constants.CANdleConstants.CANdlePort);
      

      changeAnimation(AnimationTypes.SetAll);
      CANdleConfiguration configAll = new CANdleConfiguration();
      configAll.statusLedOffWhenActive = false;
      configAll.disableWhenLOS = false;
      configAll.stripType = LEDStripType.RGB;
      configAll.brightnessScalar = 0.1;
      configAll.vBatOutputMode = VBatOutputMode.Modulated;
      candle.configAllSettings(configAll, 100);

      // m_candle.setLEDs(50, 60, 70, 80, 0, 2);
      System.out.println("CANdle initalization complete");

    }
  }

  private void changeAnimation(AnimationTypes setall) {
  }


  //tasks: Create 3 methods: LED=red, LED=blue, LED off

  public void setLEDBlue()
  {

    System.out.println("Set CANDLE blue");

    candle.setLEDs(10,10,200);
    candle.modulateVBatOutput(0.9);
  }

  public void setLEDRed()
  {
    candle.setLEDs(200,10,10);
    candle.modulateVBatOutput(0.9);
  }

  public void setLEDOff()
  {
    candle.setLEDs(0,0,0);
    candle.modulateVBatOutput(0);
  }

  public void changeAnimation(AnimationTypes toChange) {
    //m_currentAnimation = toChange;
    
    switch(toChange)
    {
        case ColorFlow:
            m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
            break;
        case Fire:
            m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
            break;
        case Larson:
            m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
            break;
        case Rainbow:
            m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
            break;
        case RgbFade:
            m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
            break;
        case SingleFade:
            m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
            break;
        case Strobe:
            m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
            break;
        case Twinkle:
            m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
            break;
        case TwinkleOff:
            m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
            break;
        case SetAll:
            m_toAnimate = null;
            break;
    }
    
    //System.out.println("Changed to " + m_currentAnimation.toString());
  }
  public void incrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
        case Fire: changeAnimation(AnimationTypes.Larson); break;
        case Larson: changeAnimation(AnimationTypes.Rainbow); break;
        case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
        case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
        case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
        case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
        case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
        case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
}
public void decrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
        case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
        case Larson: changeAnimation(AnimationTypes.Fire); break;
        case Rainbow: changeAnimation(AnimationTypes.Larson); break;
        case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
        case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
        case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
        case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
        case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
  }


}
