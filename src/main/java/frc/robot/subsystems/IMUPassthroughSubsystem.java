// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/**
 * This is not a real Subsystem, but rather a path-through class; so it does not
 * extend Subsystembase The reason - we do not want periodic methods here; if
 * you need something specific to NavX or Pigeon, put those in the respective
 * real subsystems
 * 
 * It selects the right IMU based on the information in the Constants and uses
 * methods: getPitch getRoll getYaw zeroYaw getHeading
 * 
 * All IMU Subsystems must implement IMUInterface
 * 
 * The rest of the code does not need to know whether we use NavX or Pigeon IMU.
 * It should only use this passthrough subsystem that will provide it the right
 * iinformation via methods
 */

public class IMUPassthroughSubsystem implements IMUInterface {

  private IMUInterface imu; // We will use downcasting to set this - it will point to methods either in NavX
  // or Pigeon subsystems

  /** Creates a new IMUSubsystem. */
  public IMUPassthroughSubsystem() {

    if (Constants.RobotProperties.isIMU) { // If IMU should be present - set it in Robot.java for each bot type
      if (Constants.RobotProperties.isNaVX) {
        imu = new NavXIMUSubsystem();
      } else {
        imu = new PigeonIMUSubsystem();
      }
    }
  }

  public double getPitch() {
    return imu.getPitch();
  }

  public double getRoll() {
    return imu.getRoll();
  }

  public double getYaw() {
    return imu.getYaw();
  }

  public double zeroYaw() {
    return imu.zeroYaw();
  }

  public double setYaw(double y) {
    return imu.setYaw(y);
  }

  public double getTurnRate() {
    return imu.getTurnRate();
  }

  public Rotation2d getHeading() {
    return imu.getHeading();
  }

}
