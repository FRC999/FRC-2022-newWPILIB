// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PigeonTurnToAngleLimelightXcenter extends SequentialCommandGroup {

  /** Creates a new PigeonTurnToAngleLimelightXcenter. */
  public PigeonTurnToAngleLimelightXcenter() {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Centering on Limelight"),
      new PigeonTurnPIDXLimelight(),
      new PrintCommand("Done LL centering")
    );
  }
}