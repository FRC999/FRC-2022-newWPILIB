// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NavigationControlSubsystem;

public class AutonomousTrajectoryRioCommand extends RamseteCommand {
  /** Creates a new AutonomousTrajectoryRioCommand. */

  Trajectory trajectory;

  public AutonomousTrajectoryRioCommand(Trajectory trajectory) {
    super(trajectory, () -> {
      return RobotContainer.navigationControlSubsystem.getPosition();
    }, // Lambda supplies pose for robot
        RobotContainer.navigationControlSubsystem.getRamseteController(), // Grab kinematics controller from Robot.java
        RobotContainer.navigationControlSubsystem.getFeedforward(),
        RobotContainer.navigationControlSubsystem.getKinematics(), () -> {
          return RobotContainer.navigationControlSubsystem.getWheelSpeeds();
        }, RobotContainer.navigationControlSubsystem.getLeftPidController(),
        RobotContainer.navigationControlSubsystem.getRightPidController(), (Double left, Double right) -> { // yes, I DO
                                                                                                            // mean the
                                                                                                            // object
                                                                                                            // type.
          RobotContainer.navigationControlSubsystem.setMotorVoltages(left, right);
        }, RobotContainer.navigationControlSubsystem, RobotContainer.driveSubsystem // Set requirements
    );
    this.trajectory = trajectory;
  }

  public AutonomousTrajectoryRioCommand(String alpha) {
        this(NavigationControlSubsystem.getTrajectory(alpha));
    System.out.println("initalized trajectory command: " + alpha);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.navigationControlSubsystem.resetPose(trajectory.getInitialPose());
    System.out.println("New coords" + RobotContainer.navigationControlSubsystem.getPosition());
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.navigationControlSubsystem.updateOdometer();
    RobotContainer.driveSubsystem.feed();
    super.execute();
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
