// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TESTTenFeetForward extends SequentialCommandGroup {
  /** Creates a new TESTTenFeetForward. */
  public TESTTenFeetForward() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(RobotContainer.driveSubsystem); 

    addCommands(
      new InstantCommand(RobotContainer.driveSubsystem::zeroDriveEncoders,RobotContainer.driveSubsystem),
      new InstantCommand(() -> RobotContainer.driveSubsystem.simpleMotionMagic(
        10*Constants.DriveConstants.ticksPerFoot[0],
        10*Constants.DriveConstants.ticksPerFoot[1]),
        RobotContainer.driveSubsystem)
        .until(RobotContainer.driveSubsystem::acceptableLinearError),
        new DriveStopCommand()
    );
  }
}
