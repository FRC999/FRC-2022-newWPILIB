// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TEMPShooterTestSubsystem extends SubsystemBase {
  public WPI_TalonSRX shooterMotorController1;
  public WPI_TalonSRX shooterMotorController2;

  /** Creates a new ShooterTest. */
  public TEMPShooterTestSubsystem() {

    if (Constants.RobotProperties.isTEMPShooterTest) {

      shooterMotorController1 = new WPI_TalonSRX(16);
      shooterMotorController2 = new WPI_TalonSRX(4);
    
      shooterMotorController1.setInverted(true);

    }

  }

  public void motorOn() {
    shooterMotorController1.setNeutralMode(NeutralMode.Brake);
    shooterMotorController1.set(1);
    shooterMotorController2.setNeutralMode(NeutralMode.Brake);
    shooterMotorController2.set(1);
  }

  public void motorOff() {
    shooterMotorController1.setNeutralMode(NeutralMode.Coast);
    shooterMotorController1.set(0);
    shooterMotorController2.setNeutralMode(NeutralMode.Coast);
    shooterMotorController2.set(0);
  }

  public void motorOn1() {
    shooterMotorController1.setNeutralMode(NeutralMode.Brake);
    shooterMotorController1.set(1);
  }

  public void motorOff1() {
    shooterMotorController1.setNeutralMode(NeutralMode.Coast);
    shooterMotorController1.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
