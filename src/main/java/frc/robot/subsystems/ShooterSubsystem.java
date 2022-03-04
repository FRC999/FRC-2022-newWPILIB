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
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

  private static final double CALIBRATEMOTORPOWER = 0.2;
  private static final double FULLFORWARDSPEED = 1.0;
  private static final double FULLREVERSESPEED = -0.7;

  public WPI_TalonSRX tiltMotorController;
  private WPI_TalonSRX[] wheelMotorControllers;
  private static DoubleSolenoid shooterSolenoid;

  private double shooterAnglePID; 
  private double shooterSpeedPID;
  
  private final double LIMELIGHTANGLE = 45;
  private final double LIMELIGHTHEIGHT = 85;
  private final double UPPERHUBHEIGHT = 264;

  /**
   * We suppose to zero encoder when we calibrate the shooter arm
   * But if something happens during calibration, we need to remember the ZERO (down) position of the tilt arm
   * in encoder setting
   */
  private double zeroTiltPosition;

  private final int MAXDISTANCE = 19;
  private final int MINDISTANCE = 3;

  /**
   * Artillery firing table -
   *  First dimension - distance-to target (ft)
   *  Second dimension - 0 - high goal, 1 - low goal
   *  Third dimension - 0 - angle, 1 - shooter power
   * 
   * Distance is measured from the outer edge of the upper ring to the edge of the back bumper 
   */
  private final double[][][] artilleryTable = {
    {{}},  // 0 ft - skip
    {{}},  // 1 ft - skip
    {
      {87.0, 0.547}, //2 ft high goal
      {} //skip low goal
    },
    {
      {84.0, 0.524}, //3 ft high goal
      {} //skip low goal
    },
    {
      {}, //skip high goal
      {64.0, 0.285} //5 ft low goal
    }, 
    {
      {68.0, 0.635}, //6 ft high goal
      {47.0, 0.402} //6 ft low goal
    },
    {
      {64.0, 0.602}, //7 ft high goal
      {47.0, 0.449} //7 ft low goal
    },
    {
      {65.0, 0.618}, //8 ft high goal
      {43.0, 0.465} //8 ft low goal
    },
    {
      {65.0, 0.653}, //9 ft high goal
      {45.0, 0.5} //9 ft low goal
    },
    {
      {64.0, 0.7}, //10 ft high goal
      {42.0, 0.507} //10 ft low goal
    },
    {
      {65.0, 0.708}, //11 ft high goal
      {42.0, 0.507} //11 ft low goal
    },
    {
      {62.0, 0.708}, //12 ft high goal
      {45.0, 0.543} //12 ft low goal
    },
    {
      {64.0, 0.759}, //13 ft high goal
      {46.0, 0.547} //13 ft low goal
    },
    {
      {61.0, 0.751}, //14 ft high goal
      {44.0, 0.59} //14 ft low goal
    },
    {
      {59.0, 0.787}, //15 ft high goal
      {45.0, 0.61} //15 ft low goal
    },
    {
      {}, //16 ft high goal
      {} //16 ft low goal
    },
    {
      {61.0, 0.889}, //17 ft high
      {} //skip 17 ft low
    },
    {
      {}, //18 ft high
      {}
    },
    {
      {61.0, 1.0}, //19 ft high goal
      {} //skip
    }
  };

  //array that keeps current shooter firing values, element 0 is angle, element 1 is power
  private double [] shootingSolution = new double [2];
  private boolean shootingSolutionSet = false;  // used to determine whether Shooting Solution was set successfully
  private int goalSelection = -1; //-1 = no goal selected, 0= high goal selected, 1= low goal selected
  private int attemptedDistanceSelection = 3; //3-19

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

    System.out.println("**** Shooter Initialization complete");

  }

  public void initializeWheelMotorControllers() {
    wheelMotorControllers[0].configFactoryDefault();
    wheelMotorControllers[1].configFactoryDefault();

    wheelMotorControllers[0].set(ControlMode.PercentOutput, 0);
    wheelMotorControllers[1].set(ControlMode.PercentOutput, 0);

    //wheelMotorControllers[1].follow(wheelMotorControllers[0]);

    wheelMotorControllers[0].setInverted(InvertType.InvertMotorOutput); // TODO: Check that the master motor is not inverted
    wheelMotorControllers[1].setInverted(InvertType.InvertMotorOutput);
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
   // tiltMotorController.setSensorPhase(false);


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
  }

  public void calibrateBackSlow() {
    // System.out.println("*** T B");
    tiltMotorController.setNeutralMode(NeutralMode.Brake);
    tiltMotorController.set(ControlMode.PercentOutput, CALIBRATEMOTORPOWER*(-1));
  }

  public void tiltMotorOff() {
    // System.out.println("*** T OFF");
    // leave the tilt motor in break mode so it will not just drop down
    tiltMotorController.setNeutralMode(NeutralMode.Brake);
    tiltMotorController.set(0);
  }

  /**
   * Zero Shooter Tilt Encoder
   */
  public void zeroTiltMotorEncoder() {
    tiltMotorController.setSelectedSensorPosition(0);
    // zeroTiltPosition = getTiltEncoder();
    zeroTiltPosition = 0;
  }

  public int getTiltEncoder() {
    return (int) tiltMotorController.getSelectedSensorPosition();
  }

  public double getTiltError() {
    return tiltMotorController.getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  /**
   * Tilt Shooter arm at specific angle
   * This will be extensively used by the commands that need to hold the arm at specific angle
   */
  public void tiltShooterArm(double degrees) {
    tiltMotorController.set(ControlMode.Position, zeroTiltPosition - degreesToEncoderClicks(degrees));
    
    shooterAnglePID = degrees;
    //System.out.println("**** ZT " + zeroTiltPosition + " T " + (zeroTiltPosition +0) + " E " + getTiltEncoder() + " NP " +  (zeroTiltPosition - degreesToEncoderClicks(degrees)));
    //tiltMotorController.set(ControlMode.Position,  zeroTiltPosition -540);
  }

  // Remember tilt encoder setting - used to remember the ZERO position
  public void initTiltShooterArm() {
    zeroTiltPosition = getTiltEncoder();
  }

  /**
   * Convert encoder clicks to degrees for the tilt motor
   * Note that the resolution and error of this process is 0.17 of a degree
   */
   public int degreesToEncoderClicks(double degrees) {
    return (int)(Constants.ShooterConstants.encoderUnitsPerShaftRotation * degrees / 360.0) ;
  }

  public int EncoderClicksToDegrees(int clicks) {
    return (int)(360.0 * clicks / Constants.ShooterConstants.encoderUnitsPerShaftRotation) ;
  }

  public int getTiltAngle() {
    return EncoderClicksToDegrees((int)(getTiltEncoder() - zeroTiltPosition));
  }

  public double getTiltZT() {
    return zeroTiltPosition;
  }

  public double getTiltAdjustment() {
    return RobotContainer.driveStick.getRawAxis(3)*20;
  }

  public void extendPlunger() {
    shooterSolenoid.set(Value.kForward);
  }

  public void retractPlunger() {
    shooterSolenoid.set(Value.kReverse);
  }

  public boolean isPlungerRetracted() {
    return shooterSolenoid.get() == Value.kReverse;
  }

  /**
   * manually adjust shooter wheel power using Z-tail on the turnstick
   * The -1..1 range is converted to 0..1 multiplier
   * @return double
   */
  public double shooterWheelPowerAdjustment() {
    return RobotContainer.turnStick.getRawAxis(3)*0.5 + 0.5;
  }

  public void startShooterWheelMotor() {
    startShooterWheelMotor(FULLFORWARDSPEED * shooterWheelPowerAdjustment() ); // adjust shooter power
  }

  public void startShooterWheelMotor(double power) {
    wheelMotorControllers[0].setNeutralMode(NeutralMode.Brake);
    wheelMotorControllers[1].setNeutralMode(NeutralMode.Brake);
    wheelMotorControllers[0].set(power);
    wheelMotorControllers[1].set(power);

    shooterSpeedPID = power; 
  }

  public double getShooterWheelPID() {
    return shooterSpeedPID;
  }

  public double getShooterAnglePID() {
    return shooterAnglePID;
  }

  public void startShooterWheelMotorReverse() {
    startShooterWheelMotorReverse(FULLREVERSESPEED);  // No need to adjust it
  }

  public void startShooterWheelMotorReverse(double power) {
    wheelMotorControllers[0].setNeutralMode(NeutralMode.Brake);
    wheelMotorControllers[1].setNeutralMode(NeutralMode.Brake);
    wheelMotorControllers[0].set(power);
    wheelMotorControllers[1].set(power);
  }

  public void stopShooterWheelMotor() {
    wheelMotorControllers[0].setNeutralMode(NeutralMode.Coast);
    wheelMotorControllers[1].setNeutralMode(NeutralMode.Coast);
    wheelMotorControllers[0].set(0);
    wheelMotorControllers[1].set(0);
  }

  /**
   * 
   * @param distance, goal - 0 high, 1 low
   * @return false if there is no solution, true if solution is set
   */
  public boolean setShootingSolution(int distance, int goal) {
    attemptedDistanceSelection= distance;
    shootingSolutionSet = false;
    if (distance<0 || distance>=shootingSolution.length || goal<0 || goal>1){
      goalSelection =-1;
      return false;
    }
    if (artilleryTable[distance][goal].length != 2){
      goalSelection =-1;
      return false;
    }
    goalSelection= goal;
    shootingSolution= artilleryTable[distance][goal];

    shootingSolutionSet = true;

    return true;
  }

  public boolean getShootingSolutionSet () {
    return shootingSolutionSet;
  }

  public int getGoalSelection (){
    return goalSelection;
  }

  public double[] getShootingSolution(){
    return shootingSolution;
  }

  public double getShootingSolutionAngle() {
    return shootingSolution[0];
  }

  public double getShootingSolutionPower() {
    return shootingSolution[1];
  }

  public int getAttemptedDistanceSelection(){
    return attemptedDistanceSelection;
  }

  public void nextShootingSolution(int goal){
    int newDistance = getAttemptedDistanceSelection() +1;
    if (newDistance>MAXDISTANCE){
      setShootingSolution(MINDISTANCE, goal);
    }
    else{ 
      setShootingSolution(newDistance, goal);
    }
  }

  public void previousShootingSolution(int goal){
    int newDistance = getAttemptedDistanceSelection() +1;
    if (newDistance<MINDISTANCE){
      setShootingSolution(MAXDISTANCE, goal);
    }
    else{ 
      setShootingSolution(newDistance, goal);
    }
  }

  public double getTargetDistance(){
    return Math.round( (UPPERHUBHEIGHT-LIMELIGHTHEIGHT) /
              Math.tan(
                Math.toRadians(
                  LIMELIGHTANGLE + getTargetVerticalOffset()
                )
              )
            /30.48);
  }

  // Methods that get Limelight information used in targeting

  public double getTargetVerticalOffset() { // -24.85 to 24.85 degrees
    return RobotContainer.networkTablesSubsystem.getDouble("limelight", "ty", 0);
  }
  public double getTargetHorizontalOffset() { // -29.8 to 29.8 degrees
    return RobotContainer.networkTablesSubsystem.getDouble("limelight", "tx", 0);
  }
  public boolean targetDetected() {
    return (RobotContainer.networkTablesSubsystem.getDouble("limelight", "tv", 0) == 1)?true:false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
