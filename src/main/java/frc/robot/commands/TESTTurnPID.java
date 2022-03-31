// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TESTTurnPID extends PIDCommand {

  public static final double kStabilizationP = 1;
  public static final double kStabilizationI = 0.5;
  public static final double kStabilizationD = 0;
  public static final double kTurnP = 0.7;
  public static final double kTurnI = 0.002;
  public static final double kTurnD = 0.5;
  public static final double kTurnToleranceDeg = 1;
  public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

  /** Creates a new TESTTurnPID. */
  public TESTTurnPID(double targetAngleDegrees) {
    super(
    new PIDController(kTurnP, kTurnI, kTurnD),
    // Close loop on heading
    () -> RobotContainer.imuSubsystem.getHeading().getDegrees(),
    // Set reference to target
    targetAngleDegrees,
    // Pipe output to turn robot
    output -> RobotContainer.driveSubsystem.arcadeDrive(0, -output),
    // Require the drive
    RobotContainer.driveSubsystem);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-360000, 360000);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
      .setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
