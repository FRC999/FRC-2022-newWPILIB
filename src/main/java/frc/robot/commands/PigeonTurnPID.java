// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PigeonTurnPID extends PIDCommand {

  public static final double kTurnP = 0.140;
  public static final double kTurnI = 0.010;
  public static final double kTurnD = 0.032;

  public static final double kMaxTurnRateDegPerS = 20;
  public static final double kMaxTurnAccelerationDegPerSSquared = 20;

  public static final double kTurnToleranceDeg = 1;
  public static final double kTurnRateToleranceDegPerS = 5; // degrees per second

  private double angle;

  /** Creates a new TESTPigeonTurnPID2. */
  public PigeonTurnPID( double targetAngleDegrees ) {
    super(
      new PIDController(kTurnP, kTurnI, kTurnD),
      // Close loop on heading
      RobotContainer.imuSubsystem::getYaw,
      // Set reference to target
      // targetAngleDegrees+RobotContainer.imuSubsystem.getYaw(),
      targetAngleDegrees,
      // Pipe output to turn robot
      output -> RobotContainer.driveSubsystem.arcadeDrive(0, (-1)*output*(0.2)),
      // Require the drive
      RobotContainer.driveSubsystem);

      angle = targetAngleDegrees;

      // Set the controller to be continuous (because it is an angle controller)
      getController().enableContinuousInput(-368640, 368640);
      // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
      // setpoint before it is considered as having reached the reference
      getController()
        .setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("A " + RobotContainer.imuSubsystem.getYaw() + " T " + angle);
    return getController().atSetpoint();
  }
}
