// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotProperties;

public class DriveFromHubAutonomousCommand extends CommandBase {
  /** Creates a new DriveManuallyCommand. */
  public DriveFromHubAutonomousCommand() {
   // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  double distanceValue = Constants.FieldConstants.tarmacWidestPoint;
  double encodedDistance = Constants.DriveConstants.kEncoderDistancePerPulse / distanceValue;
  //-1 should be backwards
  int sign = -1;
  int finalVal = (int)encodedDistance*sign;


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    RobotContainer.driveSubsystem.simpleMotionMagic(finalVal, finalVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(RobotContainer.driveSubsystem.getRightEncoder())-Math.abs(finalVal) <= 100 || 
       Math.abs(RobotContainer.driveSubsystem.getRightEncoder())-Math.abs(finalVal) <= 100) {
        return true;
    }
    return false;
  }
}
