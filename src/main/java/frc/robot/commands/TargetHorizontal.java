// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TargetHorizontal extends CommandBase {

  private final double Kp = -0.1;
  private final double min_command = 0.05;
  private final double MAXERROR = 1.0;

  private double heading_error ;
  private double steering_adjust = 0.0f;

  private boolean targetingComplete = false;
  

  /** Creates a new TargetingHorizontal. */
  public TargetHorizontal() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (! RobotContainer.shooterSubsystem.targetDetected()) { // end the command if you do not see the target when you start
      targetingComplete = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    heading_error = (-1)*RobotContainer.shooterSubsystem.getTargetHorizontalOffset();

    if ( Math.abs(heading_error) < MAXERROR ) {
      targetingComplete = true ;
    } else {
      if ( Math.abs(heading_error) >= MAXERROR ) {// Negative Kp with increased error
        // steering_adjust = Kp*heading_error - min_command;
        RobotContainer.driveSubsystem.manualDrive(0, Kp);
      } else {
        // steering_adjust = Kp*heading_error + min_command;
        RobotContainer.driveSubsystem.manualDrive(0, (-1)*Kp);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  //stop the robot at the end of the command or on interrupt
    RobotContainer.driveSubsystem.driveTrainBrakeMode();
    RobotContainer.driveSubsystem.manualDrive(0, 0);
    System.out.println("*** Horizontal Targeting complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetingComplete;
  }
}
