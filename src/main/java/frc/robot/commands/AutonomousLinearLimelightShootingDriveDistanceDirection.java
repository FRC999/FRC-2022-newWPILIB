// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousLinearLimelightShootingDriveDistanceDirection extends SequentialCommandGroup {

  public double SHOOTINGDISTANCE = 9; // ft

    /**
    *  Creates a new AutonomousLinearLimelightShootingDriveDistanceDirection.
    * 
    * @param double shootingDistance Distance to drive
    * @param double direction -1 drive backwards, +1 drive forward. Can also be used to reduce driving speed
    * 
    */
  public AutonomousLinearLimelightShootingDriveDistanceDirection(double shootingDistance, double direction) { // direction +1 or -1

    SHOOTINGDISTANCE = shootingDistance;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      sequence(
        new IntakeDown(),
        //new CalibrateShooterArmWithLimitSwitch(),
        //new InstantCommand(RobotContainer.shooterSubsystem::zeroTiltMotorEncoder,RobotContainer.shooterSubsystem),
        race(
          new WaitCommand(7), // max drive time; may need to be adjusted for longer drives
          new DriveUntilDistanceLimelightDirection(SHOOTINGDISTANCE, direction)
        ),
        
        new DriveStopCommand(),
        new WaitCommand(0.5),
        new ShooterOneButtonShotPreset((int)SHOOTINGDISTANCE, 0)
        //new CalibrateShooterArmWithLimitSwitch(),
        //race(
        //  new WaitCommand(5),
        //  new DriveUntil14Limelight()
        //)
       )
    );
  }
}
