// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveInterface;
import frc.robot.Constants.RobotProperties;
public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {

    // This subsystem needs to be instantiated after all device-related ones
    updateDriveSubsystemTelemetry(); // initial update for the drive subsystem

  }

  // PDP telemetry
  public void updatePDPValues() {
    SmartDashboard.putNumber("PDP Voltage", RobotContainer.pdpSubsystem.getvoltage());
    SmartDashboard.putNumber("PDP Current", RobotContainer.pdpSubsystem.getcurrent());
    SmartDashboard.putNumber("PDP power", RobotContainer.pdpSubsystem.getpower());
  }

  public void updateIMUValues() {

    SmartDashboard.putString("IMU-Y-P-R",
        String.format("%12.6f", RobotContainer.imuSubsystem.getYaw()) + "  "
            + String.format("%12.6f", RobotContainer.imuSubsystem.getPitch()) + "  "
            + String.format("%12.6f", RobotContainer.imuSubsystem.getRoll()));

  }

  public void updateDriveSubsystemTelemetry() {
    SmartDashboard.putNumber("Left Encoder Value", RobotContainer.driveSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("Left Encoder Speed", RobotContainer.driveSubsystem.getLeftEncoderSpeed());
    SmartDashboard.putNumber("Right Encoder Value", RobotContainer.driveSubsystem.getRightEncoder());
    SmartDashboard.putNumber("Right Encoder Speed", RobotContainer.driveSubsystem.getRightEncoderSpeed());
  }

  public void updateShooterValues() {
    SmartDashboard.putNumber("Tilt Encoder", RobotContainer.shooterSubsystem.getTiltEncoder());
    SmartDashboard.putNumber("Tilt Error", RobotContainer.shooterSubsystem.getTiltError() );
  }

  public void updatePotentiometerValues() {
    SmartDashboard.putNumber("Potentiometer Value", RobotContainer.potentiometerSubsystem.getPotVal());
  }

  public void updateOUValues() {
    SmartDashboard.putNumber("Z-Slider", RobotContainer.driveStick.getRawAxis(3));
  }

  public void updateColorSensorValues() {
    SmartDashboard.putNumber("Shooter Color Sensor Proximity", RobotContainer.colorSensorTestSubsystem.getObjectProximityShooter());
    SmartDashboard.putString("Shooter Color Detected", RobotContainer.colorSensorTestSubsystem.getSeenColorShooter());
    SmartDashboard.putBoolean("Shooter Red Ball Detected", RobotContainer.colorSensorTestSubsystem.isBallRedShooter());
    SmartDashboard.putBoolean("Shooter Blue Ball Detected", RobotContainer.colorSensorTestSubsystem.isBallBlueShooter());

    SmartDashboard.putNumber("Hopper Color Sensor Proximity", RobotContainer.colorSensorTestSubsystem.getObjectProximityHopper());
    SmartDashboard.putString("Hopper Color Detected", RobotContainer.colorSensorTestSubsystem.getSeenColorHopper());
    SmartDashboard.putBoolean("Hopper Red Ball Detected", RobotContainer.colorSensorTestSubsystem.isBallRedHopper());
    SmartDashboard.putBoolean("Hopper Blue Ball Detected", RobotContainer.colorSensorTestSubsystem.isBallBlueHopper());
  }

  public void ballColorChange(){
    if (RobotContainer.colorSensorTestSubsystem.isBallBlueShooter()){
      RobotContainer.candleSubsystem.setLEDBlue();
    } else if (RobotContainer.colorSensorTestSubsystem.isBallRedShooter()){
      RobotContainer.candleSubsystem.setLEDRed();
    } else {
      RobotContainer.candleSubsystem.setLEDOff();
    }
  }

  // TODO: check values to be put on smart dashboard
  public void updateAllDisplays() {

    if (Constants.RobotProperties.isIMU) {
      updateIMUValues();
    }

    updateDriveSubsystemTelemetry();

    if (Constants.RobotProperties.isShooter) {
       updateShooterValues();
    }
    if (Constants.RobotProperties.isPotentiometer) {
      updatePotentiometerValues();
    }

    if (Constants.RobotProperties.isColorSensor) {
      // updateColorSensorValues();
      // TODO: decide on the Dashboard telemetry for the color sensors
    }

    if (Constants.RobotProperties.isCANdle) {
      // ballColorChange();
      // TODO: program CANdle telemetry for the ball color change
    }

    if (RobotProperties.driveInterface == DriveInterface.ONESTICK) {
      updateOUValues();
    }

  }

  // Trajectory/kinematic driving update; updated from NavigationControlSubsystem
  public void updateMeterPrint(double left, double right) {
    SmartDashboard.putNumber("left m", left);
    SmartDashboard.putNumber("right m", right);

  }





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
