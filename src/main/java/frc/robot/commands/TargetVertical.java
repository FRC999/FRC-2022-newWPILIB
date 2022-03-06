// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class TargetVertical extends CommandBase {

  private boolean endCommand = false;
  private double distance ; // distance to the target
  private double angle ; // shooter arm angle
  int goal;

  /** Creates a new ShooterArmPositionSolution. */
  public TargetVertical(int g) {    // 0 - high, 1 -low target
    // Use addRequirements() here to declare subsystem dependencies.
    goal = g ;
    addRequirements(RobotContainer.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (! RobotContainer.shooterSubsystem.targetDetected()) { // end the command if you do not see the target when you start
      endCommand = true;
    }
    distance = RobotContainer.shooterSubsystem.getTargetDistance();

    if ( RobotContainer.shooterSubsystem.setShootingSolution( (int)Math.round(distance), goal) ) { // if firing solution for high goal exists (also set the solution parameters)
      angle = RobotContainer.shooterSubsystem.getShootingSolutionAngle();
      if (angle > 0 && angle < 90) { // invalid number from the shooting solution
        RobotContainer.shooterSubsystem.tiltShooterArm(angle);
        System.out.println("*** Shooter arm at " + angle);
      } else {
        endCommand =  true;  
      }
    } else {
      endCommand =  true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (angle > 0 && angle < 90) { // invalid number from the shooting solution
      RobotContainer.shooterSubsystem.tiltShooterArm(angle);
    } else {
      endCommand =  true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
