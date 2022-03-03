// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.RobotDriveChassisConstants;
import frc.robot.Constants.TrajectoryDriving;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

/**
 * This subsystem is used by Kinematic trajectory driving process It uses quite
 * a few other subsystems to make driving decisions
 * 
 */

public class NavigationControlSubsystem extends SubsystemBase {

  private DriveSubsystem driveSubsystem;
  private IMUPassthroughSubsystem imu;
  private DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;

  private RamseteController ramseteController = new RamseteController();
  //public RamseteController ramseteController = new RamseteController();

  private SimpleMotorFeedforward feedforward;

  private PIDController leftPidController;
  private PIDController rightPidController;

  /** Creates a new NavigationControlSubsystem. */
  public NavigationControlSubsystem(DriveSubsystem driveSubsystem, IMUPassthroughSubsystem imuSubsystem) {
    this.driveSubsystem = driveSubsystem; // Instance variable shadowed by local variable
    imu = imuSubsystem;
    kinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(RobotDriveChassisConstants.distanceBetweenWheels));
    leftPidController = new PIDController(TrajectoryDriving.trajectoryRioPidP_Value,
        TrajectoryDriving.trajectoryRioPidI_Value0, TrajectoryDriving.trajectoryRioPidD_Value0);
    rightPidController = new PIDController(TrajectoryDriving.trajectoryRioPidP_Value,
        TrajectoryDriving.trajectoryRioPidI_Value0, TrajectoryDriving.trajectoryRioPidD_Value0);
    feedforward = new SimpleMotorFeedforward(TrajectoryDriving.feedForwardStatic, TrajectoryDriving.feedForwardVelocity,
        TrajectoryDriving.feedForwardAcceleration);

    /**
     * DifferentialDriveOdometry contructor was revised since team 5190 posted their
     * video The parameters listed here were gathered from WPI documentation as well
     * as the document created by Team 8027.
     * 
     * The initial heading of the navX need not be zeroed for this to function.
     */
    odometry = new DifferentialDriveOdometry(imu.getHeading(),
        new Pose2d(TrajectoryDriving.startingPoseX, TrajectoryDriving.startingPoseY, new Rotation2d()));
  }

  /**
   * Return robot pose to starting position (as set in RobotMap) At a time there
   * was no constructor to indicate angle So, we face due east by default. There
   * is one now, so you can decide to change that by passing it angle-related
   * parameters as well
   */
  public void resetPose() {
    resetPose(new Pose2d(TrajectoryDriving.startingPoseX, TrajectoryDriving.startingPoseY, new Rotation2d()));
  }

  /**
   * This method sets the pose to the robot.
   * 
   * @param startingPose is the new position of the robot.
   */
  public void resetPose(Pose2d startingPose) {
    /**
     * This method of odometry assumes that encoders are zeroed. That fact is
     * documented, but your author missed it in his rush, and wasted precious time
     * trying to debug it.
     */
    odometry.resetPosition(startingPose, imu.getHeading());
    driveSubsystem.zeroDriveEncoders();
  }

  public void updateOdometer() {
    Rotation2d gyroAngle = imu.getHeading();

    double leftDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getLeftEncoder());
    double rightDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getRightEncoder());

    RobotContainer.smartDashboardSubsystem.updateMeterPrint(leftDistanceMeters, rightDistanceMeters);

    odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters);
  }

  public double convertEncoderTicsToMeters(int encoderTics) {
    return Units.inchesToMeters(encoderTics / driveSubsystem.getEncoderTicksPerInch());
  }

  public double convertMetersToEncoderTics(double meters) {
    return Units.metersToInches(meters) * driveSubsystem.getEncoderTicksPerInch();
  }

  /**
   * Gets the current Pose2d of the robot
   */
  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    // TODO: Check that the speed is indeed m/s and not m/100-ms

    double leftSpeed = convertEncoderTicsToMeters(driveSubsystem.getLeftEncoderSpeed());
    double rightSpeed = convertEncoderTicsToMeters(driveSubsystem.getRightEncoderSpeed());
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  /**
   * Use for RIO-based pid, sets the left- and right- motor voltages
   * 
   * @param left  Voltage for left motor, output from WPI PidController
   * @param right Voltage for right motor, output from WPI PidController
   */
  public void setMotorVoltages(double left, double right) {
    driveSubsystem.setLeftVoltage(left);
    driveSubsystem.setRightVoltage(right);
  }

  /**
   * This attempts to drive the wheels to reach the given velocities
   * 
   * @param leftSpeedMeters  speed of left side in meters per second
   * @param rightSpeedMeters speed of right side in meters per second
   */
  public void setMotorSpeeds(double leftSpeedMeters, double rightSpeedMeters) {
    /**
     * While encoder positions must be ints, velocities can be doubles Let's use
     * doubles for that bit of extra precision
     */
    double leftSpeedTics, rightSpeedTics;
    leftSpeedTics = convertMetersToEncoderTics(leftSpeedMeters);
    rightSpeedTics = convertMetersToEncoderTics(rightSpeedMeters);

    // Speeds need to be in tics per 100ms
    leftSpeedTics /= 10;
    rightSpeedTics /= 10;

    driveSubsystem.velocityPid(leftSpeedTics, rightSpeedTics);
  }

  public static Trajectory getTrajectory(String trajectoryName) {
    String trajectoryFile = "output/" + trajectoryName + ".json";
    Trajectory trajectory = new Trajectory();
    Path trajectoryPath;
    System.out.println(Filesystem.getDeployDirectory().toPath());
    System.out.println(trajectoryFile);
    trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile);
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
      System.out.println(trajectoryPath);
    }
    //System.out.println("Read Trajectory " + trajectory);
    return trajectory;
  }

  public RamseteController getRamseteController() {

    System.out.println("RC " + ramseteController);

    return ramseteController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public PIDController getRightPidController() {
    return rightPidController;
  }

  public PIDController getLeftPidController() {
    return leftPidController;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
