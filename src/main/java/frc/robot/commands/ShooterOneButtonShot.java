// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;


// This works fine s inline command as well, but we want to make this group used in Autonomous sequences as well as Teleop

public class ShooterOneButtonShot extends SequentialCommandGroup {
  /** Creates a new ShooterOneButtonShot.
   * Uses Shooter Power adjusted by Z-tail
   */
  public ShooterOneButtonShot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      deadline (
        sequence(
          new WaitCommand(1),
          new InstantCommand(RobotContainer.shooterSubsystem::extendPlunger),
          new WaitCommand(0.2),
          new InstantCommand(RobotContainer.shooterSubsystem::retractPlunger),
          new InstantCommand(RobotContainer.shooterSubsystem::stopShooterWheelMotor)
        ), // end sequence
        new InstantCommand(RobotContainer.shooterSubsystem::startShooterWheelMotor,RobotContainer.shooterSubsystem)
      ), //end deadline
      new InstantCommand(RobotContainer.shooterSubsystem::releaseTiltMotor,RobotContainer.shooterSubsystem)
    );
  }
}
