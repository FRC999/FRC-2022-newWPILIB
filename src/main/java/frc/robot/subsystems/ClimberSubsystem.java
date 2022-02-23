// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private static final double CALIBRATEMOTORPOWER = 0.2;
  private static final double FULLFORWARDSPEED = 0.7;
  private static final double FULLREVERSESPEED = -0.7;

    private WPI_TalonSRX[] climberMotorControllers;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    if (Constants.RobotProperties.isClimber) {

      // panMotorController.configFactoryDefault();
      // panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

      climberMotorControllers = new WPI_TalonSRX[] {
        new WPI_TalonSRX(Constants.ShooterConstants.shooterWheelMotorPortIDs[0]),
        new WPI_TalonSRX(Constants.ShooterConstants.shooterWheelMotorPortIDs[1]) 
      };

      initializeClimberMotorControllers();

      // Enable PID for the tilt motor
      configureClimberMotorControllerForPosition();
    }
  }

  public void initializeClimberMotorControllers() {
    climberMotorControllers[0].configFactoryDefault();
    climberMotorControllers[1].configFactoryDefault();

    climberMotorControllers[0].setInverted(InvertType.InvertMotorOutput); // TODO: Check that the master motor is not inverted

    climberMotorControllers[1].follow(climberMotorControllers[0]);
    climberMotorControllers[1].setInverted(InvertType.OpposeMaster);  // TODO: check inversion on the follower

    climberMotorControllers[0].set(ControlMode.PercentOutput, 0);
  }

  public void configureClimberMotorControllerForPosition() {


    for (int i = 0; i<=1; i++) {
		/* Config the sensor used for Primary PID and sensor direction  - ex */
    climberMotorControllers[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Constants.ClimberConstants.PID_CLIMB,
      Constants.ClimberConstants.configureTimeoutMs);

  	/* Ensure sensor is positive when output is positive - ex */
    climberMotorControllers[i].setSensorPhase(Constants.ClimberConstants.SensorPhase[i]);

    /* Configure motor neutral deadband */
    climberMotorControllers[i].configNeutralDeadband(Constants.ClimberConstants.NeutralDeadband, Constants.ClimberConstants.configureTimeoutMs);
    
    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */

    climberMotorControllers[i].configPeakOutputForward(Constants.ClimberConstants.PeakOutput, Constants.ClimberConstants.configureTimeoutMs);
    climberMotorControllers[i].configPeakOutputReverse(Constants.ClimberConstants.PeakOutput*(-1), Constants.ClimberConstants.configureTimeoutMs);
    climberMotorControllers[i].configNominalOutputForward(0, Constants.ClimberConstants.configureTimeoutMs);
    climberMotorControllers[i].configNominalOutputReverse(0, Constants.ClimberConstants.configureTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation. - ex
		 */
		climberMotorControllers[i].configAllowableClosedloopError(Constants.ClimberConstants.SLOT_0,
                                      Constants.ClimberConstants.climbDefaultAcceptableError,
                                      Constants.ClimberConstants.configureTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    /* FPID Gains for pan motor */

    climberMotorControllers[i].config_kP(Constants.ClimberConstants.SLOT_0, Constants.ClimberConstants.P_CLIMB, Constants.ClimberConstants.configureTimeoutMs);
    climberMotorControllers[i].config_kI(Constants.ClimberConstants.SLOT_0, Constants.ClimberConstants.I_CLIMB, Constants.ClimberConstants.configureTimeoutMs);
    climberMotorControllers[i].config_kD(Constants.ClimberConstants.SLOT_0, Constants.ClimberConstants.D_CLIMB, Constants.ClimberConstants.configureTimeoutMs);
    climberMotorControllers[i].config_kF(Constants.ClimberConstants.SLOT_0, Constants.ClimberConstants.F_CLIMB, Constants.ClimberConstants.configureTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match. - ex
		 */
		int absolutePosition = climberMotorControllers[i].getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.ClimberConstants.SensorPhase[i]) { absolutePosition *= -1; }
		if (Constants.ClimberConstants.MotorInvert[i]) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		climberMotorControllers[i].setSelectedSensorPosition(absolutePosition, Constants.ClimberConstants.SLOT_0, Constants.ClimberConstants.configureTimeoutMs);

    }

  } // End configureShooterMotorControllerForPosition

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
