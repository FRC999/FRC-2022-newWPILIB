// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

  private static final double CALIBRATEMOTORPOWER = 0.3;
  private static final double FULLFORWARDSPEED = 1.0;
  public static final double FULLREVERSESPEED = -0.5;

  public WPI_TalonSRX tiltMotorController;
  private WPI_TalonSRX[] wheelMotorControllers;
  private static DoubleSolenoid shooterSolenoid;

  private double shooterAnglePID; 
  private double shooterSpeedPID;
  
  private final double LIMELIGHTANGLE = 35;
  private final double LIMELIGHTHEIGHT = 85.1; // cm
  private final double UPPERHUBHEIGHT = 264;   // cm

  private final double BULLSEYEANGLE = 4.0; // Angle to target when the shot hits

  /**
   * We suppose to zero encoder when we calibrate the shooter arm
   * But if something happens during calibration, we need to remember the ZERO (down) position of the tilt arm
   * in encoder setting
   */
  private double zeroTiltPosition;

  private double rememberedTargetHorizontalOffset;

  private final int MAXDISTANCE = 19;
  private final int MINDISTANCE = 3;

  private int shooterPowerAdjustment = 0; // adjust shooter power dynamically during the game
  private final int shooterPowerAdjustmentLimit = 10; // how many steps to adjust power by
  private final int shooterPowerAdjustmentPercentage = 5; // how much to adjust shooter power per step (percent)

  /**
   * Artillery firing table -
   *  First dimension - distance-to target (ft)
   *  Second dimension - 0 - high goal, 1 - low goal
   *  Third dimension - 0 - angle, 1 - shooter power
   * 
   * Distance is measured from the outer edge of the upper ring to the edge of the back bumper 
   */
  private double[][][] artilleryTable = {
    {{}},  // 0 ft - skip
    {{}},  // 1 ft - skip
    {{}},  // 2 ft - skip
    {{}},  // 3 ft - skip
    { // 4ft
      {80.0, 0.819}, //skip high goal
      {65.0, 0.285} //4 ft low goal
    }, 
    { // 5ft
      {73.0, 0.59}, //skip high goal
      {49.0, 0.402} //5 ft low goal
    }, 
    {
      {60.0, 0.59}, //6 ft high goal
      {47.0, 0.402} //6 ft low goal
    },
    {
      {65.0, 0.629}, //7 ft high goal
      {45.0, 0.460} //7 ft low goal
    },
    {
      {65.0, 0.62745}, //8 ft high goal
      {43.0, 0.465} //8 ft low goal
    },
    {
      {65.0, 0.6259}, //9 ft high goal
      {45.0, 0.5} //9 ft low goal
    },
    {
      {63.25, 0.66545}, //10 ft high goal
      {42.0, 0.507} //10 ft low goal
    },
    {
      {61.5, 0.705}, //11 ft high goal
      {42.0, 0.507} //11 ft low goal
    },
    {
      {60.75, 0.77715}, //12 ft high goal
      {45.0, 0.543} //12 ft low goal
    },
    {
      {60.0, 0.7434}, //13 ft high goal
      {46.0, 0.547} //13 ft low goal
    },
    {
      {58.5, 0.81774}, //14 ft high goal
      {44.0, 0.59} //14 ft low goal
    },
    {
      {52.0, 0.857175}, //15 ft high goal
      {45.0, 0.61} //15 ft low goal
    },
    {
      {49.0, 0.88495}, //16 ft high goal
      {} //16 ft low goal
    },
    {
      {47.0, 0.88495}, //17 ft high
      {} //skip 17 ft low
    },
    {
      {58.83, 0.9724}, //18 ft high
      {}
    },
    {
      {58.83, 0.98}, //19 ft high goal
      {50.0, 0.751} //skip
    },
    {
      {58.83, 1.00}, //20 ft high goal
      {50.0, 0.751} //skip
    }

  };

  private double[][][] artilleryTableOrig = {
    {{}},  // 0 ft - skip
    {{}},  // 1 ft - skip
    {{}},  // 2 ft - skip
    {{}},  // 3 ft - skip
    { // 4ft
      {80.0, 0.819}, //skip high goal
      {65.0, 0.285} //4 ft low goal
    }, 
    { // 5ft
      {73.0, 0.59}, //skip high goal
      {49.0, 0.402} //5 ft low goal
    }, 
    {
      {60.0, 0.59}, //6 ft high goal
      {47.0, 0.402} //6 ft low goal
    },
    {
      {65.0, 0.629}, //7 ft high goal
      {45.0, 0.460} //7 ft low goal
    },
    {
      {65.0, 0.62745}, //8 ft high goal
      {43.0, 0.465} //8 ft low goal
    },
    {
      {65.0, 0.6259}, //9 ft high goal
      {45.0, 0.5} //9 ft low goal
    },
    {
      {63.25, 0.66545}, //10 ft high goal
      {42.0, 0.507} //10 ft low goal
    },
    {
      {61.5, 0.705}, //11 ft high goal
      {42.0, 0.507} //11 ft low goal
    },
    {
      {60.75, 0.77715}, //12 ft high goal
      {45.0, 0.543} //12 ft low goal
    },
    {
      {60.0, 0.7434}, //13 ft high goal
      {46.0, 0.547} //13 ft low goal
    },
    {
      {58.5, 0.81774}, //14 ft high goal
      {44.0, 0.59} //14 ft low goal
    },
    {
      {52.0, 0.857175}, //15 ft high goal
      {45.0, 0.61} //15 ft low goal
    },
    {
      {49.0, 0.88495}, //16 ft high goal
      {} //16 ft low goal
    },
    {
      {47.0, 0.88495}, //17 ft high
      {} //skip 17 ft low
    },
    {
      {58.83, 0.9724}, //18 ft high
      {}
    },
    {
      {58.83, 0.98}, //19 ft high goal
      {50.0, 0.751} //skip
    },
    {
      {58.83, 1.00}, //20 ft high goal
      {50.0, 0.751} //skip
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

      // retractPlunger(); // do it at the beginning of TeleOp

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

    /* // Current Limiter - commented out for now

    tiltMotorController.configPeakCurrentLimit(6, 10);          // 6 amps
    tiltMotorController.configPeakCurrentDuration(1000, 10);    // for more than 10 seconds
    tiltMotorController.configContinuousCurrentLimit(0.5, 10);  // drop the output to 0.5A
    tiltMotorController.EnableCurrentLimit(true);               // enable

    */

  } // End configurePanMotorControllerForPosition

  public void enableTiltCurrentLimit () {
    tiltMotorController.enableCurrentLimit(true);
  }

  public void disableTiltCurrentLimit () {
    tiltMotorController.enableCurrentLimit(false);
  }

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
   * This will be extensively used by the instant commands that need to hold the arm at specific angle
   * @param double degrees
   * @return void
   * Note that it does not tell you when it's done getting to the angle. Wait accordingly.
   */
  public void tiltShooterArm(double degrees) {
    tiltMotorController.set(ControlMode.Position, zeroTiltPosition - degreesToEncoderClicks(degrees));
    
    shooterAnglePID = degrees;
    //System.out.println("**** ZT " + zeroTiltPosition + " T " + (zeroTiltPosition +0) + " E " + getTiltEncoder() + " NP " +  (zeroTiltPosition - degreesToEncoderClicks(degrees)));
    //tiltMotorController.set(ControlMode.Position,  zeroTiltPosition -540);
  }

  public void tiltShooterArm0() {
    tiltShooterArm(0);
  }

  // Force-keep the tilt down
  public void tiltShooterArmDownForce() {
    tiltShooterArm(0);
  }

  // Remember tilt encoder setting - used to remember the ZERO position
  public void initTiltShooterArm() {
    zeroTiltPosition = getTiltEncoder();
  }

  /**
   * Convert degrees to encoder clicks for the tilt motor
   * Note that the resolution and error of this process is 0.17 of a degree
   */
   public int degreesToEncoderClicks(double degrees) {
    return (int)(Constants.ShooterConstants.encoderUnitsPerShaftRotation * degrees / 360.0) ;
  }

  /**
   * Convert encoder clicks to degrees for the tilt motor
   * Note that the resolution and error of this process is 0.17 of a degree
   */
  public int EncoderClicksToDegrees(int clicks) {
    return (int)(360.0 * clicks / Constants.ShooterConstants.encoderUnitsPerShaftRotation) ;
  }

  /**
   * Get current tilt angle in degrees (int) for the shooting solution
   * @return degrees
   */
  public int getTiltAngle() {
    return EncoderClicksToDegrees((int)(getTiltEncoder() - zeroTiltPosition));
  }

  public double getTiltZT() {
    return zeroTiltPosition;
  }

  /**
   * Get position of the Z-tail for the manual tilt adjustment
   * @return tiltAdjustment -1..+1 * 20 (range -20 ... +20)
   */
  public double getTiltAdjustment() {
    return RobotContainer.driveStick.getRawAxis(3)*20;
  }

  /**
   * Extend shooter plunger
   */
  public void extendPlunger() {
    shooterSolenoid.set(Value.kForward);
  }

  public boolean isClimberLockNotEngaged() {
    return shooterSolenoid.get() != Value.kForward ;
  }

  /**
   * Retract shooter plunger
   */
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

  /**
   * Set shooter wheel motor power with Z-tail adjustment
   */
  public void startShooterWheelMotor() {
    startShooterWheelMotor(FULLFORWARDSPEED * shooterWheelPowerAdjustment() ); // adjust shooter power
  }

  /**
   * Set shooter wheel motor power to a power provided as an argument
   * @param power - shooting power 0..1
   */
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
    if ( ! RobotContainer.bbl.getRawButton(Constants.OIC2022TEST.BallIntakeCalibrationSwitch)) {
      startShooterWheelMotorReverse(FULLREVERSESPEED);  // No need to adjust it
    } else { // intake calibration
      startShooterWheelMotorReverse((RobotContainer.driveStick.getRawAxis(3)-1)/2.0);
      // SmartDashboard.putNumber("Shooter Wheel Intake Power ", (RobotContainer.turnStick.getRawAxis(3)+1)/2.0);
    }
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

  public void releaseTiltMotor() {
    tiltMotorController.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**
   * 
   * @param distance, goal - 0 high, 1 low
   * @return false if there is no solution, true if solution is set
   */
  public boolean setShootingSolution(int distance, int goal) {

    System.out.println("Attempted preset " + distance + " " + goal);

    attemptedDistanceSelection = distance;
    shootingSolutionSet = false;
    if (distance<0 || distance>=artilleryTable.length || goal<0 || goal>1){
      goalSelection =-1;
      return false;
    }
    if (artilleryTable[distance][goal].length != 2){

      System.out.println ("L "+ artilleryTable[distance][goal].length);

      goalSelection =-1;
      return false;
    }
    goalSelection= goal;
    shootingSolution= artilleryTable[distance][goal];

    shootingSolutionSet = true;

    System.out.println("Preset successful");

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

  public double[] getShootingSolution(int distance) { // get angle,power for a specific shooting solution, high goal
    if (distance >0 && distance < artilleryTable.length) {
      return  artilleryTable[distance][0];
    } else {
      return new double[] {0.0,0.0};
    }
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
    int newDistance = getAttemptedDistanceSelection() -1;
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

  public double getTargetDistanceRaw(){
    return  (UPPERHUBHEIGHT-LIMELIGHTHEIGHT) /
              Math.tan(
                Math.toRadians(
                  LIMELIGHTANGLE + getTargetVerticalOffset()
                )
              )
            /30.48;
  }

  /**
   * Return TRUE if target distance matches parameter
   * @param distance - int distance to target
   * @return
   */
  public boolean isTargetDistance(int distance) {
    return Math.round(getTargetDistance()) == distance;
  }


  public boolean targetDistanceNotLess(double distance){
    return getTargetDistance()>=distance; }

  public boolean targetDistanceNotMore(double distance){
      return getTargetDistance()<=distance; }
  

  // Methods that get Limelight information used in targeting

  public double getTargetVerticalOffset() { // -24.85 to 24.85 degrees
    return RobotContainer.networkTablesSubsystem.getDouble("limelight", "ty", 0);
  }
  public double getTargetHorizontalOffset() { // -29.8 to 29.8 degrees
    return RobotContainer.networkTablesSubsystem.getDouble("limelight", "tx", 0);
  }

  /**
   * Check if Limelight detects target
   * @return true/false
   */
  public boolean isTargetDetected() {
    return (RobotContainer.networkTablesSubsystem.getDouble("limelight", "tv", 0) == 1)?true:false;
  }

  public boolean isBullsEye() {
    return isTargetDetected() && Math.abs(getTargetHorizontalOffset())< BULLSEYEANGLE ;
  }

  public void arrayDeepCopy (double source[][][], double target[][][]) {
    for (int i=0; i<source.length; i++) {
      for (int j=0; j<source[i].length; j++) {
        for (int k=0;k<source[i][j].length;k++) {
          target[i][j][k] = source[i][j][k] ;
        }
      }
    }
  }

  /** Manual shooter power adjustment DOWN during the game */
  public void shooterPowerAdjustLower() {
    if (shooterPowerAdjustment> -shooterPowerAdjustmentLimit) {
      shooterPowerAdjustment--;
      changeArtilleryTable();
      SmartDashboard.putNumber("ShooterPowerAdjustment",shooterPowerAdjustment);
    }
  }

  /** Manual shooter power adjustment UP during the game */
  public void shooterPowerAdjustHigher() {
    if (shooterPowerAdjustment< shooterPowerAdjustmentLimit) {
      shooterPowerAdjustment++;
      changeArtilleryTable();
      SmartDashboard.putNumber("ShooterPowerAdjustment",shooterPowerAdjustment);
    }
  }

  public int getShooterPowerAdjustment() {
    return shooterPowerAdjustment;
  }

  public void changeArtilleryTable() {

    for (int i=0; i<artilleryTable.length; i++) {
      for (int j=0; j<artilleryTable[i].length; j++) {
        for (int k=0;k<artilleryTable[i][j].length;k++) {
          if (k==1) { // adjust distances
            artilleryTable[i][j][k] =
              Math.min(artilleryTableOrig[i][j][k]*( 1 + (shooterPowerAdjustment*shooterPowerAdjustmentPercentage)/100.0 ),1.0) ;
          }
        }
      }
    }
  }

  public void restoreArtilleryTable() {
    arrayDeepCopy(artilleryTableOrig, artilleryTable);
    shooterPowerAdjustment = 0; // remove the adjustment
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
