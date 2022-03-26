// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveUntilDistanceLimelightDirection extends CommandBase {

  double direction;
  /** Creates a new DriveUntilDistanceLimelight. */
  public DriveUntilDistanceLimelightDirection(double d) { // direction 1 - forward; -1 - backwards
    direction = d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveSubsystem.manualDrive(0.6 * direction, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveSubsystem.manualDrive(0.6 * direction, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shooterSubsystem.targetDistanceNotLess(10);
  }
}