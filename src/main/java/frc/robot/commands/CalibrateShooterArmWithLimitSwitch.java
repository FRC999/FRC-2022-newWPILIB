// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CalibrateShooterArmWithLimitSwitch extends CommandBase {
  private final double CALIBRATIONPERCENTOUTPUT = 0.3 ; // should be reasonably slow; we do not want to hit the pan hard
  //private boolean failToCalibrate = false ; // this will be set if calibration cannot be done, for instance, if the DIO limit cannot be read

  // The Limit switch is interruptor - meaning, the circuit is closed until it's pressed.
  // That means its value will be FALSE until it's pressed
  //private DigitalInput shooterLimitSwitch; 

  /** Creates a new CalibrateShooterArmWithLimitSwitch. */
  public CalibrateShooterArmWithLimitSwitch() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);

    System.out.println("**** Start shooter arm calibration");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Setting up DIO "+Constants.ShooterConstants.shooterLimitSwitchDIOPort);

    /*  
    // initialize the limit switch
      if(shooterLimitSwitch == null) {
        try {
          shooterLimitSwitch = new DigitalInput(Constants.ShooterConstants.shooterLimitSwitchDIOPort) ;
        } catch (Exception e) { // This should not happen on a RIO, but just in case...
          System.out.println("--- Unable to check Digital Input "+Constants.ShooterConstants.shooterLimitSwitchDIOPort);
          failToCalibrate = true;
        }
      }
      System.out.println("DIO initialized");
      */

      // Start downward rotation of the shooter motor
      RobotContainer.shooterSubsystem.tiltMotorController.set(ControlMode.PercentOutput, CALIBRATIONPERCENTOUTPUT);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // drive the shooter arm down
    RobotContainer.shooterSubsystem.tiltMotorController.set(ControlMode.PercentOutput, CALIBRATIONPERCENTOUTPUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the shooter arm motor at the end of the command
    RobotContainer.shooterSubsystem.tiltMotorController.set(ControlMode.PercentOutput, 0);

    // RobotContainer.shooterSubsystem.configureTiltMotorControllerForPosition();

    // zero out the shooter arm encoder
    RobotContainer.shooterSubsystem.zeroTiltMotorEncoder();
    System.out.println("*** Calibration ended. Interrupted: " + interrupted);
    System.out.println("SLS " + RobotContainer.dioSubsystem.getSwitchStatus()) ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("*** Calibration Command finish");
    return RobotContainer.dioSubsystem.getSwitchStatus() ;
  }
}
