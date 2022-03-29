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
public class TESTPigeonTurnPID extends PIDCommand {

  public static final double kTurnP = 0.75;
  public static final double kTurnI = 0;
  public static final double kTurnD = 0.5;

  public static final double kMaxTurnRateDegPerS = 100;
  public static final double kMaxTurnAccelerationDegPerSSquared = 300;

  public static final double kTurnToleranceDeg = 1;
  public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

  /** Creates a new TESTPigeonTurnPID2. */
  public TESTPigeonTurnPID( double targetAngleDegrees ) {
    super(
      new PIDController(kTurnP, kTurnI, kTurnD),
      // Close loop on heading
      RobotContainer.imuSubsystem::getYaw,
      // Set reference to target
      targetAngleDegrees,
      // Pipe output to turn robot
      output -> RobotContainer.driveSubsystem.arcadeDrive(0, (-1)*output),
      // Require the drive
      RobotContainer.driveSubsystem);

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
    return getController().atSetpoint();
  }
}
