// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem to test motor controllers on demo board and other robots
 * This subsystem may help you to quickly test a motor in a field
 */

public class TESTMotorSubsystem extends SubsystemBase {

  private static final double CALIBRATEMOTORPOWER = 0.2;
  private static final double FULLFORWARDSPEED = 1.0;
  private static final double FULLREVERSESPEED = -1.0;

  public WPI_TalonSRX testMotorController;
  /** Creates a new TESTMotorSubsystem. */
  public TESTMotorSubsystem() {

    if (Constants.RobotProperties.isTestMotor) {
      testMotorController = new WPI_TalonSRX(Constants.TestHardwareConstants.testMotorPort);

      testMotorController.configFactoryDefault();
      testMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

      configureTestMotorControllerForPosition();

      System.out.println("*** === Test Motor CAN ID: " + Constants.TestHardwareConstants.testMotorPort);
    }
    System.out.println("** Test Motor Subsystem Configured");

  }

  public void configureTestMotorControllerForPosition() {

    // Reset Hardware - ex
    testMotorController.configFactoryDefault();

    // Configure the encoders for PID control
    //panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, Constants.ShooterConstants.PID_PAN,
    //  Constants.ShooterConstants.configureTimeoutMs);

		/* Config the sensor used for Primary PID and sensor direction  - ex */
    testMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Constants.ShooterConstants.PID_TILT,
      Constants.ShooterConstants.configureTimeoutMs);

  	/* Ensure sensor is positive when output is positive - ex */
    testMotorController.setSensorPhase(Constants.ShooterConstants.SensorPhase);

    		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. - ex 
		 */ 
		testMotorController.setInverted(Constants.ShooterConstants.MotorInvert);

    /* Configure motor neutral deadband */
    testMotorController.configNeutralDeadband(Constants.ShooterConstants.NeutralDeadband, Constants.ShooterConstants.configureTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */

    testMotorController.configPeakOutputForward(Constants.ShooterConstants.PeakOutput, Constants.ShooterConstants.configureTimeoutMs);
    testMotorController.configPeakOutputReverse(Constants.ShooterConstants.PeakOutput*(-1), Constants.ShooterConstants.configureTimeoutMs);
    testMotorController.configNominalOutputForward(0, Constants.ShooterConstants.configureTimeoutMs);
    testMotorController.configNominalOutputReverse(0, Constants.ShooterConstants.configureTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation. - ex
		 */
		testMotorController.configAllowableClosedloopError(Constants.ShooterConstants.SLOT_0,
                                      Constants.ShooterConstants.tiltDefaultAcceptableError,
                                      Constants.ShooterConstants.configureTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    /* FPID Gains for pan motor */

    testMotorController.config_kP(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.P_TILT, Constants.ShooterConstants.configureTimeoutMs);
    testMotorController.config_kI(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.I_TILT, Constants.ShooterConstants.configureTimeoutMs);
    testMotorController.config_kD(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.D_TILT, Constants.ShooterConstants.configureTimeoutMs);
    testMotorController.config_kF(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.F_TILT, Constants.ShooterConstants.configureTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match. - ex
		 */
		int absolutePosition = testMotorController.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.ShooterConstants.SensorPhase) { absolutePosition *= -1; }
		if (Constants.ShooterConstants.MotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		testMotorController.setSelectedSensorPosition(absolutePosition, Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.configureTimeoutMs);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */

    /*
    testMotorController.configClosedLoopPeriod(1, Constants.ShooterConstants.closedLoopPeriodMs, Constants.ShooterConstants.configureTimeoutMs);
    */

  } // End configuretestMotorControllerForPosition

  public void zeroTestEncoders() {
    testMotorController.setSelectedSensorPosition(0);
  }

  public int getTestMotorEncoder() {
    return (int) testMotorController.getSelectedSensorPosition();
  }

  public double getTestMotorError() {
    return testMotorController.getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  public void motorForwardSlow() {
    System.out.println("*** Start Forward SLOW");
    testMotorController.setNeutralMode(NeutralMode.Brake);
    testMotorController.set(CALIBRATEMOTORPOWER);
  }

  public void motorReverseSlow() {
    System.out.println("*** Start Reverse SLOW");
    testMotorController.setNeutralMode(NeutralMode.Brake);
    testMotorController.set((-1)*CALIBRATEMOTORPOWER);
  }

  public void motorOff() {
    System.out.println("*** Stop Test Motor");
    testMotorController.setNeutralMode(NeutralMode.Coast);
    testMotorController.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
