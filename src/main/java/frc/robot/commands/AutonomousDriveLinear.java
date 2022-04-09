// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonomousDriveLinear extends CommandBase {

  private static final boolean DEBUG = false;

  public int finalEncoderValues[];

  private double feet;

  private int tolerance;

  /** 
   * Creates a new AutonomousDriveLinear.
   * 
   * Drive forward/backwards ft specified in a parameter using MotionMagic PID on Falcons
   * 
   * @param double feetToDrive - how many feet to drive + forward - backwards
  */
  public AutonomousDriveLinear(double feetToDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    feet = feetToDrive;

    tolerance = (int)(Constants.DriveConstants.ticksPerFoot[0]/6);

    addRequirements(RobotContainer.driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    finalEncoderValues = new int[2];
    finalEncoderValues[0] = (int)(Constants.DriveConstants.ticksPerFoot[0] * feet);
    finalEncoderValues[1] = (int)(Constants.DriveConstants.ticksPerFoot[1] * feet);

    //System.out.println("**** A T L " + finalEncoderValues[0]);
    //System.out.println("**** A T R " + finalEncoderValues[1]);

    RobotContainer.driveSubsystem.zeroDriveEncoders();

    System.out.println("**** Init A T L " + finalEncoderValues[0]);
    System.out.println("**** Init A T R " + finalEncoderValues[1]);

    // Drive with MotionMagic Talon PID to calculated encoder value
    RobotContainer.driveSubsystem.simpleMotionMagic(finalEncoderValues[0],finalEncoderValues[0]);
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

}
