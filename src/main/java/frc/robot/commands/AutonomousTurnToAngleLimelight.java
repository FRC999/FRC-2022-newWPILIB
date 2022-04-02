// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonomousTurnToAngleLimelight extends CommandBase {

  private static final boolean DEBUG = true;

  public int finalEncoderValues[];

  private double angle;

  private int tolerance;

  /** Creates a new AutonomousDriveLinear. */
  public AutonomousTurnToAngleLimelight() {
    // Use addRequirements() here to declare subsystem dependencies.


    addRequirements(RobotContainer.driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    angle = (-1)*RobotContainer.shooterSubsystem.getTargetHorizontalOffset();

    finalEncoderValues = new int[2];
    if (angle>=0){
      finalEncoderValues[0] = (int)(Constants.DriveConstants.ticksPerDegree[0] * angle * (-1));
      finalEncoderValues[1] = (int)(Constants.DriveConstants.ticksPerDegree[0] * angle);
      tolerance = (int)(Constants.DriveConstants.ticksPerDegree[0]);

    } else {
      finalEncoderValues[0] = (int)(Constants.DriveConstants.ticksPerDegree[1] * angle * (-1));
      finalEncoderValues[1] = (int)(Constants.DriveConstants.ticksPerDegree[1] * angle);
      tolerance = (int)(Constants.DriveConstants.ticksPerDegree[1]/2);  
    }
    
    System.out.println("**** Turn tolerance " + tolerance);

    //System.out.println("**** A T L " + finalEncoderValues[0]);
    //System.out.println("**** A T R " + finalEncoderValues[1]);

    RobotContainer.driveSubsystem.zeroDriveEncoders();

    System.out.println("**** Init A T L " + finalEncoderValues[0]);
    System.out.println("**** Init A T R " + finalEncoderValues[1]);

    // Drive with MotionMagic Talon PID to calculated encoder value
    RobotContainer.driveSubsystem.simpleMotionMagic(finalEncoderValues[0],finalEncoderValues[1]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if (DEBUG) { // only telemetry; should not need to re-run the MotionMagic
      System.out.println("**** Exec A T L " + finalEncoderValues[0]);
      System.out.println("**** Exec A T R " + finalEncoderValues[1]);
      System.out.println("**** Exec A N L " + RobotContainer.driveSubsystem.getLeftEncoder());
      System.out.println("**** Exec A N R " + RobotContainer.driveSubsystem.getRightEncoder());
      System.out.println("**** Exec A E L " + RobotContainer.driveSubsystem.getDriveError(0));
      System.out.println("**** Exec A E R " + RobotContainer.driveSubsystem.getDriveError(1));
      System.out.println("**** Exec ATT " + angle);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.manualDrive(0, 0);
    System.out.println("*** Linear Drive done. Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    //return acceptableLinearError();
    return Math.abs(finalEncoderValues[0]-RobotContainer.driveSubsystem.getLeftEncoder())<tolerance;

    // TODO: check if we can use the PID error instead

  }

  public boolean acceptableLinearError() {
    return acceptableLinearError(0) || acceptableLinearError(1);  // 0 - error from left master motor, 1 - from the right master motor
  }
  
  public boolean acceptableLinearError(int motor){
    double closedLoopError = ((motor==0)?
        RobotContainer.driveSubsystem.getLeftEncoder() - finalEncoderValues[motor]
      : RobotContainer.driveSubsystem.getRightEncoder() - finalEncoderValues[motor]) ;

      if (DEBUG) {
        System.out.println("A C L M " + motor + " E " + closedLoopError);
      }

    return Math.abs(closedLoopError) < Constants.DriveConstants.maximumLinearError[motor] ;
  }

}
