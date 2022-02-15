// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem to test motor controllers on demo board and other robots
 */

public class TESTMotorSubsystem extends SubsystemBase {

  private static final double CALIBRATEMOTORPOWER = 0.05;
  private static final double FULLFORWARDSPEED = 1.0;
  private static final double FULLREVERSESPEED = -1.0;

  public WPI_TalonSRX testMotorController;
  /** Creates a new TESTMotorSubsystem. */
  public TESTMotorSubsystem() {

    if (Constants.RobotProperties.isTestMotor) {
      testMotorController = new WPI_TalonSRX(Constants.TestHardwareConstants.testMotorPort);

      testMotorController.configFactoryDefault();
      testMotorController.configSelectedFeedbackSensor(testMotorController, 0, 20);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
