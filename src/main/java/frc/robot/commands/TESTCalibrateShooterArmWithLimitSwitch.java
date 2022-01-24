// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class TESTCalibrateShooterArmWithLimitSwitch extends CommandBase {

  private final double CALIBRATIONPERCENTOUTPUT = -0.1 ; // should be reasonably slow; we do not want to hit the pan hard
  private boolean failToCalibrate = false ; // this will be set if calibration cannot be done, for instance, if the DIO limit cannot be read
  private DigitalInput shooterLimitSwitch;

  /** Creates a new TESTCalibrateShooterArmWithLimitSwitch. */
  public TESTCalibrateShooterArmWithLimitSwitch() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);

    System.out.println("**** Start shooter arm calibration");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Setting up DIO "+Constants.ShooterConstants.shooterLimitSwitchDIOPort);

      // initialize the limit switch
      try {
        shooterLimitSwitch = new DigitalInput(Constants.ShooterConstants.shooterLimitSwitchDIOPort) ;
      } catch (Exception e) { // This should not happen on a RIO, but just in case...
        System.out.println("--- Unable to check Digital Input "+Constants.ShooterConstants.shooterLimitSwitchDIOPort);
        failToCalibrate = true;
      }

      System.out.println("DIO initialized");

      System.out.println ("SW " + shooterLimitSwitch.get() + " FC " + failToCalibrate);

      RobotContainer.shooterSubsystem.panMotorController.set(ControlMode.PercentOutput, CALIBRATIONPERCENTOUTPUT);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive the shooter arm backwards
    RobotContainer.shooterSubsystem.panMotorController.set(ControlMode.PercentOutput, CALIBRATIONPERCENTOUTPUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the shooter arm motor at the end of the command
    RobotContainer.shooterSubsystem.panMotorController.set(ControlMode.PercentOutput, 0);

    System.out.println("*** Calibration ended. Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("C E " + (failToCalibrate || (! shooterLimitSwitch.get() ))) ;
    return failToCalibrate || (! shooterLimitSwitch.get()) ;
  }
}
