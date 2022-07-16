// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.*;
import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSubsystem extends SubsystemBase {

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
}
