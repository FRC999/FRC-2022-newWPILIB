// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IMUPassthroughSubsystem;

public class RotateCommand extends CommandBase {
  /** Creates a new AutonomousPlaceholderCommand. */
  public RotateCommand() {
      addRequirements(RobotContainer.driveSubsystem);
    //addRequirements(IMUPassthroughSubsystem);
  }

  double initialYaw;
  double targetYaw;
  double currentYaw;
  double margin = AutoConstants.turnMargin;
  double turnSpeed = AutoConstants.turnSpeed;
  boolean acceptable = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
      initialYaw = RobotContainer.imuSubsystem.getYaw();
      targetYaw = initialYaw*-1;
      //targetYaw = initialYaw+180;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      currentYaw = RobotContainer.imuSubsystem.getYaw();
    if (currentYaw > targetYaw + margin) {
        RobotContainer.driveSubsystem.manualDrive(0, -turnSpeed);
    } else if (currentYaw < targetYaw - margin) {
        RobotContainer.driveSubsystem.manualDrive(0, turnSpeed);
    } else {
        RobotContainer.driveSubsystem.manualDrive(0, turnSpeed);
        acceptable = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return acceptable;
  }
}
