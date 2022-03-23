// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotProperties;

public class DriveManuallyCommand extends CommandBase {
  /** Creates a new DriveManuallyCommand. */
  public DriveManuallyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveSubsystem.driveTrainBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double move = 0, turn = 0;

    switch (RobotProperties.driveInterface) {
      case SPLITSTICK:
        move = RobotContainer.driveStick.getY() * (-1);
        turn = RobotContainer.turnStick.getX();
        break;
      case THREENEWBB: // add 2 sticks
        move = RobotContainer.driveStick.getY() * (-1);
        turn = RobotContainer.turnStick.getX();
        break;
      case ONESTICK:
        move = RobotContainer.driveStick.getY() * (-1);
        turn = RobotContainer.driveStick.getX();
        break;
      case XBOXANDSTICK: // 1 stick and XBOX controller are created
      case XBOX: // just the XBOX controller
        move = RobotContainer.xboxController.getRightY();
        turn = RobotContainer.xboxController.getRightX();
        break;
      default:  // assume at least driveStick and turnStick by default
        move = RobotContainer.driveStick.getY() * (-1);
        turn = RobotContainer.turnStick.getX();
      break;
    }

    //System.out.println("*** MDr");

    RobotContainer.driveSubsystem.manualDrive(move, turn * Constants.DriveConstants.turnAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
