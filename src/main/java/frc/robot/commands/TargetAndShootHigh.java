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
public class TargetAndShootHigh extends SequentialCommandGroup {


  /** Creates a new TargetingVertical. */
  public TargetAndShootHigh() {

    double distance = RobotContainer.shooterSubsystem.getTargetDistance() ; // distance to the target

    if ( RobotContainer.shooterSubsystem.setShootingSolution((int)(Math.round(distance)), 0) ) { // if firing solution for high goal exists (also set the solution parameters)

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());

      /**
       * 1. Perform horizontal targeting
       * 2. While doing vertical targeting, do the following:
       *  1) Wait 2 seconds (allowing vertical targeting to complete)
       *  2) Shoot
       * The command is done when the shooting is complete
       */

      addCommands(
        sequence (
          new TargetHorizontal(),
          deadline (
            sequence (
              new WaitCommand(2),
              new ShooterOneButtonShot()
            ),
            new TargetVertical(0) // tilt shooter arm to the degree indicated in the shooter solution
          ),  // end deadline
          new InstantCommand(RobotContainer.shooterSubsystem::releaseTiltMotor,RobotContainer.shooterSubsystem)
        )
      );

    }
  }
}
