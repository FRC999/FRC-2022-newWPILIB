// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TESTShooterArmPosition extends CommandBase {

  private double targetPosition = RobotContainer.shooterSubsystem.getTiltEncoder();

  /** Creates a new TESTShooterArmPosion. */
  public TESTShooterArmPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // set the rotation position
    targetPosition = RobotContainer.shooterSubsystem.getTiltEncoder() + degreesToEncoderClicks(90);
    RobotContainer.shooterSubsystem.tiltMotorController.set(ControlMode.Position, targetPosition);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Test joystic slider for manual elevation change in PID
    double adjustedPosition = targetPosition + (1 - RobotContainer.driveStick.getRawAxis(3)) * 250 ;
    RobotContainer.shooterSubsystem.tiltMotorController.set(ControlMode.Position, adjustedPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public int degreesToEncoderClicks(double degrees) {
    return (int)(Constants.ShooterConstants.encoderUnitsPerShaftRotation * degrees / 360.0) ;
  }
}
