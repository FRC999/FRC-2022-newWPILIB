// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public static final double CALIBRATEMOTORPOWER = 0.05;

  public WPI_TalonSRX panMotorController;
  public WPI_TalonSRX limitSwitchMotorController;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    if (Constants.RobotProperties.isShooter) {
      panMotorController = new WPI_TalonSRX(Constants.ShooterConstants.tiltMotorPortID);

      // panMotorController.configFactoryDefault();
      // panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

      // Enable PID for the tilt motor
      configurePanMotorControllerForPosition();

      limitSwitchMotorController = new WPI_TalonSRX(Constants.ShooterConstants.limitSwitchMotorPortID);
      limitSwitchMotorController.configFactoryDefault();

			panMotorController.configForwardLimitSwitchSource(	RemoteLimitSwitchSource.RemoteTalonSRX,
													        LimitSwitchNormal.NormallyClosed,
													        limitSwitchMotorController.getDeviceID(),
													        Constants.ShooterConstants.configureTimeoutMs);

      panMotorController.configReverseLimitSwitchSource(	RemoteLimitSwitchSource.RemoteTalonSRX,
													        LimitSwitchNormal.NormallyClosed,
													        limitSwitchMotorController.getDeviceID(),
													        Constants.ShooterConstants.configureTimeoutMs);

			System.out.println("Limit Switches locally enabled.");

    }

  }

  public void configurePanMotorControllerForPosition() {

    // Reset Hardware - ex
    panMotorController.configFactoryDefault();

    // Configure the encoders for PID control
    //panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, Constants.ShooterConstants.PID_PAN,
    //  Constants.ShooterConstants.configureTimeoutMs);

		/* Config the sensor used for Primary PID and sensor direction  - ex */
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Constants.ShooterConstants.PID_PAN,
      Constants.ShooterConstants.configureTimeoutMs);

  	/* Ensure sensor is positive when output is positive - ex */
    panMotorController.setSensorPhase(Constants.ShooterConstants.SensorPhase);

    		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. - ex 
		 */ 
		panMotorController.setInverted(Constants.ShooterConstants.MotorInvert);

    /* Configure motor neutral deadband */
    panMotorController.configNeutralDeadband(Constants.ShooterConstants.NeutralDeadband, Constants.ShooterConstants.configureTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */

    panMotorController.configPeakOutputForward(Constants.ShooterConstants.PeakOutput, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configPeakOutputReverse(Constants.ShooterConstants.PeakOutput*(-1), Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configNominalOutputForward(0, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configNominalOutputReverse(0, Constants.ShooterConstants.configureTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation. - ex
		 */
		panMotorController.configAllowableClosedloopError(Constants.ShooterConstants.SLOT_0,
                                      Constants.ShooterConstants.panDefaultAcceptableError,
                                      Constants.ShooterConstants.configureTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    /* FPID Gains for pan motor */

    panMotorController.config_kP(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.P_PAN, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.config_kI(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.I_PAN, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.config_kD(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.D_PAN, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.config_kF(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.F_PAN, Constants.ShooterConstants.configureTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match. - ex
		 */
		int absolutePosition = panMotorController.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.ShooterConstants.SensorPhase) { absolutePosition *= -1; }
		if (Constants.ShooterConstants.MotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		panMotorController.setSelectedSensorPosition(absolutePosition, Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.configureTimeoutMs);

    /* 

    panMotorController.configClosedLoopPeakOutput(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.PeakOutput_0, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configAllowableClosedloopError(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.panDefaultAcceptableError, Constants.ShooterConstants.configureTimeoutMs);

    panMotorController.configMotionAcceleration(Constants.ShooterConstants.panAcceleration, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configMotionCruiseVelocity(Constants.ShooterConstants.panCruiseVelocity, Constants.ShooterConstants.configureTimeoutMs);
    panMotorController.configMotionSCurveStrength(Constants.ShooterConstants.panSmoothing);
    */

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */

    /*
    panMotorController.configClosedLoopPeriod(1, Constants.ShooterConstants.closedLoopPeriodMs, Constants.ShooterConstants.configureTimeoutMs);
    */

  } // End configurePanMotorControllerForPosition

  public void calibrateForwardSlow() {

    System.out.println("*** T F");
    panMotorController.setNeutralMode(NeutralMode.Brake);
    panMotorController.set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER);
    //panMotorController.setInverted(false);
    //panMotorController.set(CALIBRATEMOTORPOWER);
  }

  public void calibrateBackSlow() {
    System.out.println("*** T B");
    panMotorController.setNeutralMode(NeutralMode.Brake);
    panMotorController.set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER * (-1));
    //panMotorController.setInverted(true);
    //panMotorController.set(CALIBRATEMOTORPOWER);
  }

  public void tiltMotorOff() {
    System.out.println("*** T OFF");
    panMotorController.setNeutralMode(NeutralMode.Coast);
    panMotorController.set(0);
  }

  /**
   * Zero Shooter Tilt Encoder
   */
  public void zeroSRXEncoders() {
    panMotorController.setSelectedSensorPosition(0);
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
