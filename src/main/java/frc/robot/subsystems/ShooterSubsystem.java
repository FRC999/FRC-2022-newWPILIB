// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private static final double CALIBRATEMOTORPOWER = 0.05;
  private static final double FULLFORWARDSPEED = 1.0;
  private static final double FULLREVERSESPEED = -1.0;

  public WPI_TalonSRX tiltMotorController;
  private WPI_TalonSRX[] wheelMotorControllers;
  private static DoubleSolenoid shooterSolenoid;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    if (Constants.RobotProperties.isShooter) {
      tiltMotorController = new WPI_TalonSRX(Constants.ShooterConstants.tiltMotorPortID);

      // panMotorController.configFactoryDefault();
      // panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

      wheelMotorControllers = new WPI_TalonSRX[] {
        new WPI_TalonSRX(Constants.ShooterConstants.shooterWheelMotorPortIDs[0]),
        new WPI_TalonSRX(Constants.ShooterConstants.shooterWheelMotorPortIDs[1]) 
      };

      initializeWheelMotorControllers();

      shooterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.ShooterConstants.shooterSolenoidChannels[0],
        Constants.ShooterConstants.shooterSolenoidChannels[1]);

      retractPlunger();

      // Enable PID for the tilt motor
      configureTiltMotorControllerForPosition();
    }

  }

  public void initializeWheelMotorControllers() {
    wheelMotorControllers[0].configFactoryDefault();
    wheelMotorControllers[1].configFactoryDefault();

    wheelMotorControllers[0].set(ControlMode.PercentOutput, 0);
    wheelMotorControllers[1].set(ControlMode.PercentOutput, 0);

    wheelMotorControllers[1].follow(wheelMotorControllers[0]);

    wheelMotorControllers[0].setInverted(InvertType.None); // TODO: Check that the master motor is not inverted
    wheelMotorControllers[1].setInverted(InvertType.OpposeMaster);
  }

  public void configureTiltMotorControllerForPosition() {

    // Reset Hardware - ex
    tiltMotorController.configFactoryDefault();

    // Configure the encoders for PID control
    //panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, Constants.ShooterConstants.PID_PAN,
    //  Constants.ShooterConstants.configureTimeoutMs);

		/* Config the sensor used for Primary PID and sensor direction  - ex */
    tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Constants.ShooterConstants.PID_TILT,
      Constants.ShooterConstants.configureTimeoutMs);

  	/* Ensure sensor is positive when output is positive - ex */
    tiltMotorController.setSensorPhase(Constants.ShooterConstants.SensorPhase);

    		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. - ex 
		 */ 
		tiltMotorController.setInverted(Constants.ShooterConstants.MotorInvert);

    /* Configure motor neutral deadband */
    tiltMotorController.configNeutralDeadband(Constants.ShooterConstants.NeutralDeadband, Constants.ShooterConstants.configureTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */

    tiltMotorController.configPeakOutputForward(Constants.ShooterConstants.PeakOutput, Constants.ShooterConstants.configureTimeoutMs);
    tiltMotorController.configPeakOutputReverse(Constants.ShooterConstants.PeakOutput*(-1), Constants.ShooterConstants.configureTimeoutMs);
    tiltMotorController.configNominalOutputForward(0, Constants.ShooterConstants.configureTimeoutMs);
    tiltMotorController.configNominalOutputReverse(0, Constants.ShooterConstants.configureTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation. - ex
		 */
		tiltMotorController.configAllowableClosedloopError(Constants.ShooterConstants.SLOT_0,
                                      Constants.ShooterConstants.tiltDefaultAcceptableError,
                                      Constants.ShooterConstants.configureTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    /* FPID Gains for pan motor */

    tiltMotorController.config_kP(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.P_TILT, Constants.ShooterConstants.configureTimeoutMs);
    tiltMotorController.config_kI(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.I_TILT, Constants.ShooterConstants.configureTimeoutMs);
    tiltMotorController.config_kD(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.D_TILT, Constants.ShooterConstants.configureTimeoutMs);
    tiltMotorController.config_kF(Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.F_TILT, Constants.ShooterConstants.configureTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match. - ex
		 */
		int absolutePosition = tiltMotorController.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.ShooterConstants.SensorPhase) { absolutePosition *= -1; }
		if (Constants.ShooterConstants.MotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		tiltMotorController.setSelectedSensorPosition(absolutePosition, Constants.ShooterConstants.SLOT_0, Constants.ShooterConstants.configureTimeoutMs);

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

    // System.out.println("*** T F");
    tiltMotorController.setNeutralMode(NeutralMode.Brake);
    tiltMotorController.set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER);
    //panMotorController.setInverted(false);
    //panMotorController.set(CALIBRATEMOTORPOWER);
  }

  public void calibrateBackSlow() {
    // System.out.println("*** T B");
    tiltMotorController.setNeutralMode(NeutralMode.Brake);
    tiltMotorController.set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER * (-1));
    //panMotorController.setInverted(true);
    //panMotorController.set(CALIBRATEMOTORPOWER);
  }

  public void tiltMotorOff() {
    // System.out.println("*** T OFF");
    tiltMotorController.setNeutralMode(NeutralMode.Coast);
    tiltMotorController.set(0);
  }

  /**
   * Zero Shooter Tilt Encoder
   */
  public void zeroSRXEncoders() {
    tiltMotorController.setSelectedSensorPosition(0);
  }

  public int getTiltEncoder() {
    return (int) tiltMotorController.getSelectedSensorPosition();
  }

  public double getTiltError() {
    return tiltMotorController.getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  public void pushBall() {
    shooterSolenoid.set(Value.kForward);
  }

  public void retractPlunger() {
    shooterSolenoid.set(Value.kReverse);
  }

  public boolean isPlungerRetracted() {
    return shooterSolenoid.get() == Value.kReverse;
  }

  public void shootBall() {
    shootBall(FULLFORWARDSPEED);
  }

  public void shootBall(double power) {
    wheelMotorControllers[0].setNeutralMode(NeutralMode.Brake);
    wheelMotorControllers[0].set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
