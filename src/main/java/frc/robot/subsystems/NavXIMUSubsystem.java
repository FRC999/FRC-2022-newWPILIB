// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXIMUSubsystem extends SubsystemBase implements IMUInterface {

  private AHRS NavX;

  /** Creates a new NavXSubsystem. */
  public NavXIMUSubsystem() {
    try {
      NavX = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      // TODO: We need to set up a drivers system output system: perhaps Robot.java
      // should catch this exception
      // Or we could output to the driver system directly, as they do in their example
      // DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(),
      // true);
    }

  }

  /**
   * Gets the pitch of the robot (X axis rotation) (pitch is rotation around the
   * horizontal axis perpendicular to straight forward)
   * 
   * @return The pitch of the robot
   */
  public double getPitch() {
    return NavX.getPitch();
  }

  /**
   * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
   * axis that goes straight forward)
   * 
   * @return
   */
  public double getRoll() {
    return NavX.getRoll();
  }

  /**
   * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
   * robot is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getYaw() {
    return NavX.getYaw();
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double temporaryDouble = NavX.getYaw();
    NavX.zeroYaw();
    return temporaryDouble;
  }

  public double setYaw(double y) {
    double temporaryDouble = NavX.getYaw();
    NavX.zeroYaw();
    NavX.setAngleAdjustment(y);
    return temporaryDouble;
  }

  /**
   * Provide heading in degrees with the angle increasing clockwise hence the
   * negative value of getAngle
   * 
   * @return
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-NavX.getAngle());
  }

  public AHRS getNavX() {
    return NavX;
  }

  public double getTurnRate() {
    return NavX.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
