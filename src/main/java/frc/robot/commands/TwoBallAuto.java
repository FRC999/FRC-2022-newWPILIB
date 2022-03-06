// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;


// This works fine s inline command as well, but we want to make this group used in Autonomous sequences as well as Teleop

public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new ShooterOneButtonShot. */
  public TwoBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        sequence(
          new IntakeDown(),
          new TargetHorizontal(),
          new TargetVertical(0),
          new ShooterOneButtonShot(),
          //new Rotate180(),
          new DriveFromHubAutonomousCommand(),
          //new IntakePickUpCommand(),
          //newIntakeUp(),
          //new Rotate180(),
          new IntakeDown(),
          new TargetHorizontal(),
          new TargetVertical(0),
          new ShooterOneButtonShot()
        )
    );
  }
}
