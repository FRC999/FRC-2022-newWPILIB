// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotDriveChassisConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  public WPI_TalonFX[] rightDriveTalonFX = new WPI_TalonFX[DriveConstants.rightMotorPortID.length];
  public WPI_TalonFX[] leftDriveTalonFX = new WPI_TalonFX[DriveConstants.leftMotorPortID.length];
  public DifferentialDrive drive;

  // For isOnTarget
  boolean wasOnTarget = false;
  int withinAcceptableErrorLoops = 0;

  public DriveSubsystem() {

    /**
     * create objects for the left-side and right-side motors reset controllers to
     * defaults setup followers set controller orientation set encoder phase
     */

    System.out.println("Primary Motor - Left " + DriveConstants.leftMotorPortID[0] + " Primary Motor - Right " + DriveConstants.rightMotorPortID[0] + " Number of motors per side " + DriveConstants.rightMotorPortID.length);

    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor] = new WPI_TalonFX(DriveConstants.leftMotorPortID[motor]);
      leftDriveTalonFX[motor].configFactoryDefault(); // reset the controller to defaults
      if (motor == 0) { // setup master
        leftDriveTalonFX[motor].set(ControlMode.PercentOutput, 0); // set the motor to Percent Output with Default of 0
        leftDriveTalonFX[motor].setInverted(Constants.DriveConstants.MotorInvert[0]);
      } else { // setup followers
        leftDriveTalonFX[motor].follow(leftDriveTalonFX[0]);
        leftDriveTalonFX[motor].setInverted(InvertType.FollowMaster); // set green lights when going forward
        System.out.println("Left Follower " + motor);
      }

      leftDriveTalonFX[motor].setSafetyEnabled(false);
    }

    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor] = new WPI_TalonFX(DriveConstants.rightMotorPortID[motor]);
      rightDriveTalonFX[motor].configFactoryDefault(); // reset the controller to defaults

      if (motor == 0) { // setup master
        rightDriveTalonFX[motor].set(ControlMode.PercentOutput, 0); // set the motor to Percent Output with Default of 0
        rightDriveTalonFX[motor].setInverted( Constants.DriveConstants.MotorInvert[1]);
      } else { // setup followers
        rightDriveTalonFX[motor].follow(rightDriveTalonFX[0]);
        rightDriveTalonFX[motor].setInverted(InvertType.FollowMaster); // set green lights when going forward
        System.out.println("Right Follower " + motor);
      }

      rightDriveTalonFX[motor].setSafetyEnabled(false);
    }

    // Engage brake mode
    driveTrainBrakeMode();

    // TEST Coast Mode
    //driveTrainCoastMode();

    zeroDriveEncoders();

    drive = new DifferentialDrive(leftDriveTalonFX[0], rightDriveTalonFX[0]);

    // Prevent WPI drivetrain class from inverting input for right side motors
    // because we already inverted them
    // The new 2022 version of the libraries will stop inverting the motors by
    // default anyway
    // drive.setRightSideInverted(false);

    configureSimpleMagic();

    zeroDriveEncoders(); // Needs to be done after configuring Motion Magic

  }

  /**
   * Telemetry on the Shuffleboard for the DriveSubsystem
   */

  public void feed() {
    drive.feed();
  }

  /**
   * Engage Brake Mode
   */
  public void driveTrainBrakeMode() {
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor].setNeutralMode(NeutralMode.Brake);
    }
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor].setNeutralMode(NeutralMode.Brake);
    }
  }

  public void driveTrainCoastMode() {
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor].setNeutralMode(NeutralMode.Coast);
    }
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor].setNeutralMode(NeutralMode.Coast);
    }
  }

  /**
   * This gets the number of encoder tics in a given inch
   * 
   * @return encoder tics in double form, for precision(tm)
   */
  public double getEncoderTicksPerInch() {
    // tics per rotation / number of inches per rotation * gearReduction
    return RobotDriveChassisConstants.encoderUnitsPerShaftRotation
        / (RobotDriveChassisConstants.wheelDiameter * Math.PI) * RobotDriveChassisConstants.encoderGearReduction;
  }

  public void setLeftVoltage(double voltage) {
    rightDriveTalonFX[0].setVoltage(voltage);
  }

  public void setRightVoltage(double voltage) {
    rightDriveTalonFX[0].setVoltage(voltage);
  }

  public double deadbandMove(double move) {
    if (Math.abs(move) >= DriveConstants.deadbandY) {
      if (move > 0) {
        move = (move - DriveConstants.deadbandY) / (1 - DriveConstants.deadbandY);
      } else {
        move = (move + DriveConstants.deadbandY) / (1 - DriveConstants.deadbandY);
      }
    } else {
      move = 0;
    }
    return move;
  }

  public double deadbandTurn(double turn) {
    if (Math.abs(turn) >= DriveConstants.deadbandX) {
      if (turn > 0) {
        turn = (turn - DriveConstants.deadbandX) / (1 - DriveConstants.deadbandX);
      } else {
        turn = (turn + DriveConstants.deadbandX) / (1 - DriveConstants.deadbandX);
      }
    } else {
      turn = 0;
    }
    return turn;
  }

  public void manualDrive(double move, double turn) {
    
    // If joysticks will prove to be too sensitive near the center, turn on the deadband driving
    
    // drive.arcadeDrive(deadbandMove(move), deadbandTurn(turn));
    // System.out.println("D X "+move + " Y " + turn);
    drive.arcadeDrive(move, turn);
  }

  /** Get the number of tics moved by the left encoder */
  public int getLeftEncoder() {
    return (int) leftDriveTalonFX[0].getSelectedSensorPosition();
  }

    /** Get the number of tics moved by the left encoder */
    public int getLeftEncoder(int motor) {
      return (int) leftDriveTalonFX[motor].getSelectedSensorPosition();
    }

  /** Get the number of tics moved by the left encoder */
  public int getRightEncoder() {
    return (int) rightDriveTalonFX[0].getSelectedSensorPosition();
  }

  // Make sure to adjust Units-of-measure if needed
  // The RAW output is "number of ticks per 100ms", so you may need to convert it
  // into m/s
  public int getLeftEncoderSpeed() {
    return (int) leftDriveTalonFX[0].getSelectedSensorVelocity();
  }

  public int getRightEncoderSpeed() {
    return (int) rightDriveTalonFX[0].getSelectedSensorVelocity();
  }

  public void zeroDriveEncoders() {

    rightDriveTalonFX[0].setSelectedSensorPosition(0);
    leftDriveTalonFX[0].setSelectedSensorPosition(0);
    //rightDriveTalonFX[0].setSelectedSensorPosition(DriveConstants.SLOT_0, DriveConstants.kPIDLoopIdx, DriveConstants.configureTimeoutMs);
    //leftDriveTalonFX[0].setSelectedSensorPosition(DriveConstants.SLOT_0, DriveConstants.kPIDLoopIdx, DriveConstants.configureTimeoutMs);
    //rightDriveTalonFX[0].getSensorCollection().setIntegratedSensorPosition(0, DriveConstants.configureTimeoutMs);
    //leftDriveTalonFX[0].getSensorCollection().setIntegratedSensorPosition(0, DriveConstants.configureTimeoutMs);

    // driveTrainCoastMode(); // TODO: figure out why this was introduced in 2020 - removed

    // TEST C2020
    rightDriveTalonFX[1].setSelectedSensorPosition(0);
    leftDriveTalonFX[1].setSelectedSensorPosition(0);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void resetToFactoryDefaults() {
    leftDriveTalonFX[0].configFactoryDefault();
    rightDriveTalonFX[0].configFactoryDefault();
  }

  public void safetyOff() {
    leftDriveTalonFX[0].setSafetyEnabled(false);
    leftDriveTalonFX[0].setExpiration(100);
    rightDriveTalonFX[0].setSafetyEnabled(false);
    rightDriveTalonFX[0].setExpiration(100);
  }


  /**
   * We configure drivetrain for SimpleMagic here. Note that the drivetrain will
   * work without it. But using it may allow you to smooth the driving pattern and
   * make the robot more responsive
   */
  public void configureSimpleMagic() {

    // We assume we have the same number of left motors as we have the right ones
    //for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
    for (int motor = 0; motor < 1; motor++) {

      /* Config the sensor used for Primary PID and sensor direction  - ex */
      leftDriveTalonFX[motor].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
        Constants.DriveConstants.kPIDLoopIdx,
        Constants.DriveConstants.configureTimeoutMs);

      /* Config the sensor used for Primary PID and sensor direction  - ex */
      rightDriveTalonFX[motor].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
        DriveConstants.kPIDLoopIdx,
        DriveConstants.configureTimeoutMs);

      /* Configure motor neutral deadband */
      rightDriveTalonFX[motor].configNeutralDeadband(DriveConstants.NeutralDeadband, DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].configNeutralDeadband(DriveConstants.NeutralDeadband, DriveConstants.configureTimeoutMs);

      // Reset motors to defaults - already done during initialization
      //rightDriveTalonFX[motor].configFactoryDefault();
      //leftDriveTalonFX[motor].configFactoryDefault();
      leftDriveTalonFX[motor].setSensorPhase(Constants.DriveConstants.SensorPhase[0]);
      rightDriveTalonFX[motor].setSensorPhase(Constants.DriveConstants.SensorPhase[1]);

      leftDriveTalonFX[motor].setInverted(Constants.DriveConstants.MotorInvert[0]);
      rightDriveTalonFX[motor].setInverted( Constants.DriveConstants.MotorInvert[1]);



      /* Set status frame periods to ensure we don't have stale data */
      
      rightDriveTalonFX[motor].setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,
          DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,
          DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,
          DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,
          DriveConstants.configureTimeoutMs);
      //rightDriveTalonFX[motor].setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20,
      //    DriveConstants.configureTimeoutMs);
      //leftDriveTalonFX[motor].setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20,
      //    DriveConstants.configureTimeoutMs);

      /**
      * Max out the peak output (for all modes). However you can limit the output of
      * a given PID object with configClosedLoopPeakOutput().
      */
      leftDriveTalonFX[motor].configPeakOutputForward(+1.0, DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].configPeakOutputReverse(-1.0, DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].configNominalOutputForward(0, DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].configNominalOutputReverse(0, DriveConstants.configureTimeoutMs);

      rightDriveTalonFX[motor].configPeakOutputForward(+1.0, DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].configPeakOutputReverse(-1.0, DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].configNominalOutputForward(0, DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].configNominalOutputReverse(0, DriveConstants.configureTimeoutMs);      
      
      /* FPID Gains for each side of drivetrain */
      leftDriveTalonFX[motor].selectProfileSlot(DriveConstants.SLOT_0, DriveConstants.kPIDLoopIdx);
      leftDriveTalonFX[motor].config_kP(DriveConstants.SLOT_0, DriveConstants.motionMagicPidP_Value,
        DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].config_kI(DriveConstants.SLOT_0, DriveConstants.motionMagicPidI_Value,
        DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].config_kD(DriveConstants.SLOT_0, DriveConstants.motionMagicPidD_Value,
        DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].config_kF(DriveConstants.SLOT_0, DriveConstants.motionMagicPidF_Value,
        DriveConstants.configureTimeoutMs);

      /*
      leftDriveTalonFX[motor].config_IntegralZone(DriveConstants.SLOT_0, DriveConstants.Izone_0,
        DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].configClosedLoopPeakOutput(DriveConstants.SLOT_0, DriveConstants.PeakOutput_0,
        DriveConstants.configureTimeoutMs);
      leftDriveTalonFX[motor].configAllowableClosedloopError(DriveConstants.SLOT_0, 5, DriveConstants.configureTimeoutMs);
      */

      rightDriveTalonFX[motor].selectProfileSlot(DriveConstants.SLOT_0, DriveConstants.kPIDLoopIdx);
      rightDriveTalonFX[motor].config_kP(DriveConstants.SLOT_0, DriveConstants.motionMagicPidP_Value,
        DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].config_kI(DriveConstants.SLOT_0, DriveConstants.motionMagicPidI_Value,
        DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].config_kD(DriveConstants.SLOT_0, DriveConstants.motionMagicPidD_Value,
        DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].config_kF(DriveConstants.SLOT_0, DriveConstants.motionMagicPidF_Value,
        DriveConstants.configureTimeoutMs);

      /* 
      rightDriveTalonFX[motor].config_IntegralZone(DriveConstants.SLOT_0, DriveConstants.Izone_0,
        DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].configClosedLoopPeakOutput(DriveConstants.SLOT_0, DriveConstants.PeakOutput_0,
        DriveConstants.configureTimeoutMs);
      rightDriveTalonFX[motor].configAllowableClosedloopError(DriveConstants.SLOT_0, 5, DriveConstants.configureTimeoutMs);
      */

    }

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    //rightDriveTalonFX[0].configClosedLoopPeriod(0, DriveConstants.closedLoopPeriodMs,
    //    DriveConstants.configureTimeoutMs);
    //leftDriveTalonFX[0].configClosedLoopPeriod(0, DriveConstants.closedLoopPeriodMs,
    //    DriveConstants.configureTimeoutMs);

    /* Motion Magic Configurations */

    /**
     * Need to replace numbers with real measured values for acceleration and cruise
     * vel.
     */
    leftDriveTalonFX[0].configMotionAcceleration(DriveConstants.motionMagicAcceleration,
        DriveConstants.configureTimeoutMs);
    leftDriveTalonFX[0].configMotionCruiseVelocity(2* DriveConstants.motionMagicCruiseVelocity,
        DriveConstants.configureTimeoutMs);
    leftDriveTalonFX[0].configMotionSCurveStrength(DriveConstants.motionMagicSmoothing);

    rightDriveTalonFX[0].configMotionAcceleration(DriveConstants.motionMagicAcceleration,
        DriveConstants.configureTimeoutMs);
    rightDriveTalonFX[0].configMotionCruiseVelocity(2*DriveConstants.motionMagicCruiseVelocity,
        DriveConstants.configureTimeoutMs);
    rightDriveTalonFX[0].configMotionSCurveStrength(DriveConstants.motionMagicSmoothing);

    leftDriveTalonFX[0].setSelectedSensorPosition(0, 0, DriveConstants.configureTimeoutMs);
    rightDriveTalonFX[0].setSelectedSensorPosition(0, 0, DriveConstants.configureTimeoutMs);

  } // End configureDriveTrainControllersForSimpleMagic


  public void simpleMotionMagic(int leftEncoderVal, int rightEncoderVal) {
    // Test method that moves robot forward a given number of wheel rotations
    leftDriveTalonFX[0].set(TalonFXControlMode.MotionMagic, leftEncoderVal);
    rightDriveTalonFX[0].set(TalonFXControlMode.MotionMagic, rightEncoderVal);
  }

  public void simpleMotionMagicSetFollower() {
    rightDriveTalonFX[0].follow(leftDriveTalonFX[0]);
  }

  /**
   * Used by Trajectory driving This attempts to drive the wheels to reach the
   * given velocities
   * 
   * @param leftSpeedTics  speed of left side in encoder tics per 100ms
   * @param rightSpeedTics speed of right side in encoder tics per 100ms
   */
  public void velocityPid(double leftSpeedTics, double rightSpeedTics) {
    leftDriveTalonFX[0].set(ControlMode.Velocity, leftSpeedTics);
    rightDriveTalonFX[0].set(ControlMode.Velocity, rightSpeedTics);
  }

  public int angleToTicks(double angle, int motor){
    return (int)(angle*Constants.DriveConstants.ticksPerDegree[motor]);
  }

  public int DistanceToTicks(double angle, int motor){
    return (int)(angle*Constants.DriveConstants.ticksPerFoot[motor]);
  }

  public boolean acceptableLinearError() {
    return acceptableLinearError(0) || acceptableLinearError(1);
  }
  
  public boolean acceptableLinearError(int motor){
    double closedLoopError = ((motor==0)?leftDriveTalonFX[0].getClosedLoopError():rightDriveTalonFX[0].getClosedLoopError()) ;

    System.out.println("A C L M " + motor + " E " + closedLoopError);

    return Math.abs(closedLoopError) < Constants.DriveConstants.maximumLinearError[motor] ;
  }

  public boolean acceptableAngleError() {
    return acceptableDegreeError(0) || acceptableDegreeError(1);
  }

  public boolean acceptableDegreeError(int motor){
    double closedLoopError = ((motor==0)?leftDriveTalonFX[0].getClosedLoopError():rightDriveTalonFX[0].getClosedLoopError()) ;
    return Math.abs(closedLoopError) < Constants.DriveConstants.maximumAngleError[motor] ;
  }

  public double getDriveError(int motor) {
    return ((motor==0)?leftDriveTalonFX[0].getClosedLoopError():rightDriveTalonFX[0].getClosedLoopError());// Returns the PID error for Pan motion control;
  }


  // isOnTarget methods that we see in 2021 code were removed as they seem to be
  // related to the
  // "manual" trajectory driving rather than Kinematic driving one

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
