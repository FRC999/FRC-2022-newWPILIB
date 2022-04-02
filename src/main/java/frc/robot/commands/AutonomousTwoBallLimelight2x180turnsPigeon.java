// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTwoBallLimelight2x180turnsPigeon extends SequentialCommandGroup {

  private final double FIRSTBALLSHOOTINGDISTANCE = 7.0;
  private final double DISTANCETOPICKSECONBALL = 4;
  private final double SECONDBALLSHOOTINGDISTANCE = 9; // ft

  /** Creates a new AutonomousTwoBallLimelight2x180turnsPigeon. */
  public AutonomousTwoBallLimelight2x180turnsPigeon() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      sequence(
        new InstantCommand(RobotContainer.shooterSubsystem::zeroTiltMotorEncoder,RobotContainer.shooterSubsystem), // set 0 for the shooter tilt
        new InstantCommand(RobotContainer.imuSubsystem::zeroYaw),
        new AutonomousLinearLimelightShootingDriveDistanceDirection(FIRSTBALLSHOOTINGDISTANCE, -1), // First ball
        new PrintCommand("*** First Ball Done"),
        new DriveStopCommand(),
        new InstantCommand(() -> RobotContainer.shooterSubsystem.tiltShooterArm(0)), // lower shooter arm
        new AutonomousDriveLinear(1.0), // go forward 1 ft - shoud be at 6ft now
        new DriveStopCommand(),
        new PigeonTurnToAngle(180), // turn around
        new DriveStopCommand(),
        new AutonomousPickupBallWhileDrivingStraight(DISTANCETOPICKSECONBALL), // 11 ft
        new PigeonTurnToAngle(0), // turn around; now about 9 ft to the target - intake is 3ft 
        //new AutonomousTurnToAngleLimelight(),
        //new AutonomousTurnToAngleLimelight(),
        new PigeonTurnToAngleLimelightXcenter(),
        new PigeonTurnToAngleLimelightXcenter(),
        new ShooterOneButtonShotPreset((int)SECONDBALLSHOOTINGDISTANCE, 0),

        new DriveStopCommand()
     )
    );
  }
}
