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
public class ShootOverIntake extends SequentialCommandGroup {
  /** Creates a new ShooterOneButtonShotPreset. */
  public ShootOverIntake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      sequence(
        new IntakeUp(),
        new InstantCommand(() -> RobotContainer.shooterSubsystem.tiltShooterArm(5)),
        new WaitCommand(0.5),
        deadline (
          sequence(
            new WaitCommand(1),
            new InstantCommand(RobotContainer.shooterSubsystem::extendPlunger),
            new WaitCommand(0.2),
            new InstantCommand(RobotContainer.shooterSubsystem::retractPlunger),
            new InstantCommand(RobotContainer.shooterSubsystem::stopShooterWheelMotor)
          ), // end sequence
          new InstantCommand(() -> RobotContainer.shooterSubsystem.startShooterWheelMotor(1.0))
        ), //end deadline
        new IntakeDown()
      )
    );
  }
}
