// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.DescriptiveMath;
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
    SmartDashboard.putNumber("Tilt Angle", RobotContainer.shooterSubsystem.getTiltAngle() );
    SmartDashboard.putNumber("Shooter Power Adjustment", RobotContainer.shooterSubsystem.shooterWheelPowerAdjustment() );
    SmartDashboard.putNumber("Tilt ZT", RobotContainer.shooterSubsystem.getTiltZT() );
    SmartDashboard.putNumber("Shooter Angle", RobotContainer.shooterSubsystem.getShooterAnglePID() );
    SmartDashboard.putNumber("Shooter Power", RobotContainer.shooterSubsystem.getShooterWheelPID() );
    SmartDashboard.putNumber("Shooter Goal", RobotContainer.shooterSubsystem.getGoalSelection());
    SmartDashboard.putNumberArray("Shooter A/P", RobotContainer.shooterSubsystem.getShootingSolution());
    SmartDashboard.putNumber("Shooter Attempted Distance", RobotContainer.shooterSubsystem.getAttemptedDistanceSelection());
  }

  public void updateTestMotorValues() {
    SmartDashboard.putNumber("TEST Encoder", RobotContainer.testMotorSubsystem.getTestMotorEncoder());
    SmartDashboard.putNumber("TEST Error", RobotContainer.testMotorSubsystem.getTestMotorError() );
  }


  public void updatePotentiometerValues() {
    SmartDashboard.putNumber("Potentiometer Value", RobotContainer.potentiometerSubsystem.getPotVal());
  }

  public void updateOUValues() {
    SmartDashboard.putNumber("Z-Slider", RobotContainer.driveStick.getRawAxis(3));
  }

  public void updateLimelightValues() {
    SmartDashboard.putNumber("Limelight ty", RobotContainer.networkTablesSubsystem.getDouble("limelight", "ty", 0));
    SmartDashboard.putNumber("Limelight thor", RobotContainer.networkTablesSubsystem.getDouble("limelight", "thor", 0));
    // SmartDashboard.putNumber("Limelight mean", DescriptiveMath.trimmean(RobotContainer.targetingCamera.getSamples(), 10));
    SmartDashboard.putNumber("Limelight distance", RobotContainer.shooterSubsystem.getTargetDistance());
  }

  public void updateColorSensorValues() {
    SmartDashboard.putNumber("Shooter Color Sensor Proximity", RobotContainer.colorSensorSubsystem.getObjectProximityShooter());
    SmartDashboard.putString("Shooter Color Detected", RobotContainer.colorSensorSubsystem.getSeenColorShooter());
    SmartDashboard.putBoolean("Shooter Red Ball Detected", RobotContainer.colorSensorSubsystem.isBallRedShooter());
    SmartDashboard.putBoolean("Shooter Blue Ball Detected", RobotContainer.colorSensorSubsystem.isBallBlueShooter());

    SmartDashboard.putNumber("Hopper Color Sensor Proximity", RobotContainer.colorSensorSubsystem.getObjectProximityHopper());
    SmartDashboard.putString("Hopper Color Detected", RobotContainer.colorSensorSubsystem.getSeenColorHopper());
    SmartDashboard.putBoolean("Hopper Red Ball Detected", RobotContainer.colorSensorSubsystem.isBallRedHopper());
    SmartDashboard.putBoolean("Hopper Blue Ball Detected", RobotContainer.colorSensorSubsystem.isBallBlueHopper());
  }

  public void ballColorChange(){
    if (RobotContainer.colorSensorSubsystem.isBallBlueShooter()){
      RobotContainer.candleSubsystem.setLEDBlue();
    } else if (RobotContainer.colorSensorSubsystem.isBallRedShooter()){
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

    if (Constants.RobotProperties.isTestMotor) {
      updateTestMotorValues();
   }

    if (Constants.RobotProperties.isPotentiometer) {
      updatePotentiometerValues();
    }

    if (Constants.RobotProperties.isColorSensor) {
      updateColorSensorValues();
      // TODO: decide on the Dashboard telemetry for the color sensors
    }

    if (Constants.RobotProperties.isCANdle) {
      // ballColorChange();
      // TODO: program CANdle telemetry for the ball color change
    }

    if (RobotProperties.driveInterface == DriveInterface.ONESTICK) {
      updateOUValues();
    }

    updateLimelightValues();

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
