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

public class ClimberSubsystem extends SubsystemBase {

  private static final double CALIBRATEMOTORPOWER = 1.0;
  private static final double FULLFORWARDSPEED = 0.7;
  private static final double FULLREVERSESPEED = -0.7;
  private static final int CLIMBEREXTENDED = 80000;
  private static final int CLIMBERRETRACTED = 0;

  private boolean climberLimitsOverride;

  private static DoubleSolenoid thirdArmSolenoid;

  private int[] zeroEncoder = new int[] {0,0};
  private int[] maxEncoder = new int[] {10000,10000};


  private WPI_TalonSRX[] climberMotorControllers;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    if (Constants.RobotProperties.isClimber) {

      // panMotorController.configFactoryDefault();
      // panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

      climberMotorControllers = new WPI_TalonSRX[] {
        new WPI_TalonSRX(Constants.ClimberConstants.climberMotorPortIDs[0]),
        new WPI_TalonSRX(Constants.ClimberConstants.climberMotorPortIDs[1]) 
      };

      initializeClimberMotorControllers();

      // Enable PID for the climber motors
      configureClimberMotorControllerForPosition();

      // Zero climber encoders
      zeroClimberMotorEncoders();

      for (int i=0;i<=1;i++) {
        zeroEncoder[i] = getEncoder(i);
      }

      thirdArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.ClimberConstants.climberSolenoidChannels[0],
        Constants.ClimberConstants.climberSolenoidChannels[1]); 
    }


    System.out.println("**** Climber Initialization complete");
  }

  public void initializeClimberMotorControllers() {

    for (int i=0;i<=1;i++) {
      climberMotorControllers[i].configFactoryDefault();

      climberMotorControllers[i].setInverted(Constants.ClimberConstants.MotorInvert[i]);

      //climberMotorControllers[1].follow(climberMotorControllers[0]);  // not setting FOLLOWER mode since one climber is slower than the other.
      //climberMotorControllers[1].setInverted(InvertType.FollowMaster); 

      climberMotorControllers[i].set(ControlMode.PercentOutput, 0);
    }
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

  public void zeroClimberMotorEncoders() {
    for (int i = 0; i<=1; i++) {
      climberMotorControllers[i].setSelectedSensorPosition(0);
    }
  }

  public void retractClimber(int motor) {
    climberMotorControllers[motor].set(CLIMBERRETRACTED);
  }

  public void extendClimber(int motor) {
    climberMotorControllers[motor].set(CLIMBEREXTENDED);
  }

  public void calibrateForwardSlow() {

    // System.out.println("*** C F");
    climberMotorControllers[0].setNeutralMode(NeutralMode.Brake);
    climberMotorControllers[0].set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER);
    climberMotorControllers[1].setNeutralMode(NeutralMode.Brake);
    climberMotorControllers[1].set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER);

  }

  public void calibrateBackSlow() {
    // System.out.println("*** C B");
    climberMotorControllers[0].setNeutralMode(NeutralMode.Brake);
    climberMotorControllers[0].set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER*(-1));
    climberMotorControllers[1].setNeutralMode(NeutralMode.Brake);
    climberMotorControllers[1].set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER*(-1));

  }

  public void calibrateForwardSlow(int motor) {
      climberMotorControllers[motor].setNeutralMode(NeutralMode.Brake);
      climberMotorControllers[motor].set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER);
  }

  public void calibrateBackSlow(int motor) {
      climberMotorControllers[motor].setNeutralMode(NeutralMode.Brake);
      climberMotorControllers[motor].set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER*(-1));
  }

  public int getEncoder(int encoder) {  // should be 0 or 1
    return (int) climberMotorControllers[encoder].getSelectedSensorPosition();
  }

  public double getEncoderError(int encoder) {
    return climberMotorControllers[encoder].getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  // Make sure to adjust Units-of-measure if needed
  // The RAW output is "number of ticks per 100ms", so you may need to convert it
  // into m/s
  public int getEncoderSpeed(int encoder) { // should be 0 or 1
    return (int) climberMotorControllers[encoder].getSelectedSensorVelocity();
  }

  public void climberMotorOff() {
    // System.out.println("*** T OFF");
    // leave the tilt motor in break mode so it will not just drop down
    climberMotorControllers[0].setNeutralMode(NeutralMode.Brake);
     // leave the tilt motor in break mode so it will not just drop down
     climberMotorControllers[1].setNeutralMode(NeutralMode.Brake);
    climberMotorControllers[0].set(0);
    climberMotorControllers[1].set(0);
  }

  public void climberMotorOff(int motor) {
    climberMotorControllers[motor].setNeutralMode(NeutralMode.Brake);
    climberMotorControllers[motor].set(0);
  }

  public void extendThirdArm() {
    thirdArmSolenoid.set(Value.kForward);
  }

  public void retractThirdArm() {
    thirdArmSolenoid.set(Value.kReverse);
  }

  public void climberLimitsOverrideSet() {
    climberLimitsOverride = true;
  }

  public void climberLimitsOverrideUnset() {
    climberLimitsOverride = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
