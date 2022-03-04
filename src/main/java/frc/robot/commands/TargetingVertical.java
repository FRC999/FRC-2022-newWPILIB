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
public class TargetingVertical extends SequentialCommandGroup {


  /** Creates a new TargetingVertical. */
  public TargetingVertical() {

    double distance = RobotContainer.shooterSubsystem.getTargetDistance() ; // distance to the target

    if ( RobotContainer.shooterSubsystem.setFiringSolution((int)distance, 0) ) { // if firing solution for high goal exists (also set the solution parameters)

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(

        deadline (
          sequence (
            new WaitCommand(2),
            new ShooterOneButtonShot()
          ),
          new InstantCommand( () -> RobotContainer.shooterSubsystem.tiltShooterArm( 
            (RobotContainer.shooterSubsystem.getShootingSolution())[0]) ) // tilt shooter arm to the degree indicated in the shooter solution
        )  // end deadline
      );

    }
  }
}
