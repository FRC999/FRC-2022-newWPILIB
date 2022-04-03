// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class PigeonIMUSubsystem extends SubsystemBase implements IMUInterface {

  private WPI_TalonSRX pigeyTalonSRX;
  private PigeonIMU pidgey;
  private double[] xyz = new double[3]; // so not to allocate one every time

  /** Creates a new PigeonIMUSubsystem. */
  public PigeonIMUSubsystem() {
    try {
      pigeyTalonSRX = new WPI_TalonSRX(Constants.PigeonIMU.pigeonIMUId);
      pidgey = new PigeonIMU(pigeyTalonSRX);
    } catch (RuntimeException ex) {
      System.out.println("**** Pigeon was expected on CAN " + Constants.PigeonIMU.pigeonIMUId + "but could not be initialized. Check CAN.");
      // TODO: exception handling
      // at least it will not fail if PidgeonIMU is not there
    }
  }

  /**
   * Gets the pitch of the robot (X axis rotation) (pitch is rotation around the
   * horizontal axis perpendicular to straight forward)
   * 
   * @return The pitch of the robot
   */
  public double getPitch() {
    double[] ypr = new double[3];
    pidgey.getYawPitchRoll(ypr);
    return ypr[1];
  }

  /**
   * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
   * axis that goes straight forward)
   * 
   * @return
   */
  public double getRoll() {
    double[] ypr = new double[3];
    pidgey.getYawPitchRoll(ypr);
    return ypr[2];
  }

  /**
   * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
   * robot is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getYaw() {
    double[] ypr = new double[3];
    pidgey.getYawPitchRoll(ypr);
    return ypr[0];
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double temporaryDouble = getYaw();
    pidgey.setYaw(0);
    return temporaryDouble;
  }

  public double setYaw(double y) {
    double temporaryDouble = getYaw();
    pidgey.setYaw(y);
    return temporaryDouble;
  }

  /**
   * Provide heading in degrees with the angle increasing clockwise hence the
   * negative value of getAngle
   * 
   * @return
   */
  public Rotation2d getHeading() {
    pidgey.getAccumGyro(xyz);
    return Rotation2d.fromDegrees(xyz[2]); // return accumulated Z-axis
    // TODO: test whether the degrees returned by Pidgey clockwise are positive or
    // negative
  }

  public double getTurnRate() {
    double[] ypr = new double[3];
    pidgey.getRawGyro(ypr);
    return -ypr[2];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
