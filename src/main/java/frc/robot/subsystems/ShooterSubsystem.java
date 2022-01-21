// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public static final double CALIBRATEMOTORPOWER = 0.3;

  public static WPI_TalonSRX panMotorController;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    if (Constants.RobotProperties.isShooter) {
      panMotorController = new WPI_TalonSRX(Constants.ShooterConstants.tiltMotorPortID);
      panMotorController.configFactoryDefault();
      panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

      // Enable PID for the tilt motor
      // configurePanMotorControllerForPosition();
    }

  }

  public void configurePanMotorControllerForPosition() {
    // Configure the encoders for PID control
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, Constants.ShooterConstants.PID_PAN,
      Constants.ShooterConstants.configureTimeoutMs);

    /* Configure motor neutral deadband */
    panMotorController.configNeutralDeadband(Constants.ShooterConstants.NeutralDeadband, Constants.ShooterConstants.configureTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    panMotorController.configPeakOutputForward(+1, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configPeakOutputReverse(-1, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configNominalOutputForward(0, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configNominalOutputReverse(0, Constants.ShooterConstants.configureTimeoutMs);

    /* FPID Gains for pan motor */

    panMotorController.config_kP(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.P_PAN, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.config_kI(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.I_PAN, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.config_kD(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.D_PAN, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.config_kF(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.F_PAN, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configClosedLoopPeakOutput(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.PeakOutput_0, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configAllowableClosedloopError(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.panDefaultAcceptableError, Constants.ShooterConstants.configureTimeoutMs);

    panMotorController.configMotionAcceleration(Constants.ShooterConstants.panAcceleration, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configMotionCruiseVelocity(Constants.ShooterConstants.panCruiseVelocity, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configMotionSCurveStrength(Constants.ShooterConstants.panSmoothing);
    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    panMotorController.configClosedLoopPeriod(1, Constants.ShooterConstants.closedLoopPeriodMs, Constants.ShooterConstants.configureTimeoutMs);

  } // End configurePanMotorControllerForPosition

  public void calibrateForwardSlow() {
    panMotorController.setNeutralMode(NeutralMode.Brake);
    panMotorController.set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER);
  }

  public void calibrateBackSlow() {
    panMotorController.setNeutralMode(NeutralMode.Brake);
    panMotorController.set(ControlMode.PercentOutput, -CALIBRATEMOTORPOWER);
  }

  public void tiltMotorOff() {
    panMotorController.setNeutralMode(NeutralMode.Coast);
    panMotorController.set(0);
  }

  public int getPanEncoder() {
    return (int) panMotorController.getSelectedSensorPosition();
  }

  public double getPanError() {
    return panMotorController.getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
