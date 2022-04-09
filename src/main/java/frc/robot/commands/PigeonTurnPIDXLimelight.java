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
public class PigeonTurnPIDXLimelight extends PIDCommand {
  /** Creates a new PigeonTurnPIDXLimelight. */
   //public static final double kTurnP = 0.140;
  //public static final double kTurnP = 0.070;
  public static final double kTurnP = 0.220;
  public static final double kTurnI = 0.010;
  public static final double kTurnD = 0.032;

  public static final double kMaxTurnRateDegPerS = 20;
  public static final double kMaxTurnAccelerationDegPerSSquared = 20;

  public static final double kTurnToleranceDeg = 3;
  public static final double kTurnRateToleranceDegPerS = 5; // degrees per second

  private double angle;
  private static double targetAngleDegrees=0;
  private double ll;

  /** Creates a new TESTPigeonTurnPID2. */
  public PigeonTurnPIDXLimelight() {
    super(
      new PIDController(kTurnP, kTurnI, kTurnD),
      // Close loop on heading
      RobotContainer.imuSubsystem::getYaw,
      // Set reference to target
      // targetAngleDegrees+RobotContainer.imuSubsystem.getYaw(),
      //targetAngleDegrees,
      0,
      // Pipe output to turn robot
      output -> 
        //{},
         RobotContainer.driveSubsystem.arcadeDrive(0, (-1)*output*(0.2)),
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

  @Override
  public void initialize() {

    ll = (-1)*RobotContainer.shooterSubsystem.getTargetHorizontalOffset();

    RobotContainer.imuSubsystem.setYaw(-ll);

    angle = RobotContainer.imuSubsystem.getYaw();
    targetAngleDegrees =  angle + (-1)*RobotContainer.shooterSubsystem.getTargetHorizontalOffset();
    System.out.println("TEST2 TA " + targetAngleDegrees + "Y " + angle);

    getController().setSetpoint(0);


  }

  //@Override
  //public void execute() {
    //getController().setSetpoint(targetAngleDegrees);
    //System.out.println("SP " + getController().getSetpoint());
  //}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println(
    //  "PE " + getController().getPositionError() + " TA " + targetAngleDegrees + " A " + RobotContainer.imuSubsystem.getYaw()
    //);
    return getController().atSetpoint();
  }
}
