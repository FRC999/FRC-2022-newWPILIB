// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonomousDriveLinear extends CommandBase {

  public int finalEncoderValues[];
  /** Creates a new AutonomousDriveLinear. */
  public AutonomousDriveLinear(double feet) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.driveSubsystem);

    finalEncoderValues = new int[2];
    finalEncoderValues[0] = (int)(Constants.DriveConstants.ticksPerFoot[0] * feet);
    finalEncoderValues[1] = (int)(Constants.DriveConstants.ticksPerFoot[1] * feet);

    System.out.println("**** A T L " + finalEncoderValues[0]);
    System.out.println("**** A T R " + finalEncoderValues[1]);

    RobotContainer.driveSubsystem.zeroDriveEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.driveSubsystem.resetToFactoryDefaults();
    RobotContainer.driveSubsystem.configureSimpleMagic();
    RobotContainer.driveSubsystem.safetyOff();

    RobotContainer.driveSubsystem.zeroDriveEncoders();

    System.out.println("**** Init A T L " + finalEncoderValues[0]);
    System.out.println("**** Init A T R " + finalEncoderValues[1]);

    //RobotContainer.driveSubsystem.simpleMotionMagicSetFollower();

    //RobotContainer.driveSubsystem.leftDriveTalonFX[0].setSafetyEnabled(false);
    //RobotContainer.driveSubsystem.rightDriveTalonFX[0].setSafetyEnabled(false);

    RobotContainer.driveSubsystem.simpleMotionMagic(finalEncoderValues[0],finalEncoderValues[1]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //System.out.println("**** Exec A T L " + finalEncoderValues[0]);
    //System.out.println("**** Exec A T R " + finalEncoderValues[1]);

    //RobotContainer.driveSubsystem.simpleMotionMagic(finalEncoderValues[0],finalEncoderValues[1]);
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
    return acceptableLinearError();
  }

  public boolean acceptableLinearError() {
    return acceptableLinearError(0) || acceptableLinearError(1);
  }
  
  public boolean acceptableLinearError(int motor){
    double closedLoopError = ((motor==0)?
        RobotContainer.driveSubsystem.getLeftEncoder() - finalEncoderValues[motor]
      : RobotContainer.driveSubsystem.getRightEncoder() - finalEncoderValues[motor]) ;

      System.out.println("A C L M " + motor + " E " + closedLoopError);

    return Math.abs(closedLoopError) < Constants.DriveConstants.maximumLinearError[motor] ;
  }

}
