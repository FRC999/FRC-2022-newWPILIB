// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootEnemyBall extends SequentialCommandGroup {
  /** Creates a new ShootEnemyBall. */
  public ShootEnemyBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeUp(),
      new WaitCommand(0.5),
      new InstantCommand(() -> RobotContainer.shooterSubsystem.tiltShooterArm(0), RobotContainer.shooterSubsystem),
      new WaitCommand(0.5),
      new ShooterOneButtonShot()
    );
  }
}