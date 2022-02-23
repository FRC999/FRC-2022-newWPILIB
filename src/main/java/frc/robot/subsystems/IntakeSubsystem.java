// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX intakeMotorController;
  private DoubleSolenoid intakeSolenoid;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    if(Constants.RobotProperties.isIntake) {
      intakeMotorController = new WPI_TalonSRX(Constants.IntakeConstants.intakeMotorPort);
      intakeMotorController.setInverted(true);
      intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.IntakeConstants.intakeSolenoidChannel[0],
        Constants.IntakeConstants.intakeSolenoidChannel[1]);

      // Lower intake at the start of the game
      raiseIntake();
    }
    System.out.println("**** Intake Initialization complete");
  }

  public void lowerIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  public void raiseIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  public boolean isIntakeUp() {
    return intakeSolenoid.get() == Value.kReverse;
  }

  public boolean isIntakeDown() {
    return intakeSolenoid.get() == Value.kForward;
  }

  public void rotateIntakeForward() {
    intakeMotorController.setNeutralMode(NeutralMode.Brake);
    intakeMotorController.set(Constants.IntakeConstants.intakeForwardSpeed);
  }

  public void rotateIntakeReverse() {
    intakeMotorController.setNeutralMode(NeutralMode.Brake);
    intakeMotorController.set(Constants.IntakeConstants.intakeReverseSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotorController.setNeutralMode(NeutralMode.Coast);
    intakeMotorController.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
