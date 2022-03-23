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
public class ShooterOneButtonShotPreset extends SequentialCommandGroup {
  /** Creates a new ShooterOneButtonShotPreset. */
  public ShooterOneButtonShotPreset(int distance, int goal) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(RobotContainer.shooterSubsystem); // interrupt other shooter commands if needed
    addCommands(
      sequence(
        new InstantCommand(() -> RobotContainer.shooterSubsystem.setShootingSolution(distance,goal)),
        // start shooter motor as soon as we know the shooting solution speed
        new InstantCommand(() -> RobotContainer.shooterSubsystem.tiltShooterArm( (RobotContainer.shooterSubsystem.getShootingSolution())[0])),
        new WaitCommand(0.5),
        new InstantCommand(() -> RobotContainer.shooterSubsystem.startShooterWheelMotor((RobotContainer.shooterSubsystem.getShootingSolution())[1])),
        new WaitCommand(0.5), // TODO - check the wheel speed, adjust this time to make sure the shooter wheel gets up to speed
        new InstantCommand(RobotContainer.shooterSubsystem::extendPlunger),
        new WaitCommand(0.2),
        new InstantCommand(RobotContainer.shooterSubsystem::retractPlunger),
        new InstantCommand(RobotContainer.shooterSubsystem::stopShooterWheelMotor)
      ) // not bringing it down automatically, in case we have another ball in the hopper or the shooting did not clear the ball
    );
  }
}
