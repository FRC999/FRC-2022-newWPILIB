// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/**
 * Intake down
 * Drive back for up to 2 seconds, until the Limelight shows you're 9 ft from the target
 * Stop
 * Wait 0.5s
 * Shoot 9ft preset
 */

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousBackLimelight9ft extends SequentialCommandGroup {

  public double SHOOTINGDISTANCE = 9; // ft

  /** Creates a new AutonomousBackLimelight9ft. */
  public AutonomousBackLimelight9ft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      sequence(
        new IntakeDown(),
        race(
          new WaitCommand(2),
          new InstantCommand(() -> RobotContainer.driveSubsystem.manualDrive(-0.5, 0),RobotContainer.driveSubsystem)
            .until(() -> RobotContainer.shooterSubsystem.targetDistanceNotLess(SHOOTINGDISTANCE)) )
        ),
        new DriveStopCommand(),
        new WaitCommand(0.5),
        new ShooterOneButtonShotPreset((int)SHOOTINGDISTANCE, 0)
    );
  }
}
