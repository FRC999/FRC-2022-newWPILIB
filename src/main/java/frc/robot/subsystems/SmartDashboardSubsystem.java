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

  /**
   * The following two items are used to reduce frequency of telemetry generation
   * We will update some telemetry once per MAXCOUNT loops.
   */
  private final int MAXCOUNT=10;
  private int counter=0;

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
    SmartDashboard.putNumber("Left Motor Error", RobotContainer.driveSubsystem.getDriveError(0));
    SmartDashboard.putNumber("Right Motor Error", RobotContainer.driveSubsystem.getDriveError(1));

    //SmartDashboard.putNumber("Left1 Encoder Value", RobotContainer.driveSubsystem.getLeftEncoder(1));
  }

  public void updateShooterValues() {
    SmartDashboard.putNumber("Tilt Encoder", RobotContainer.shooterSubsystem.getTiltEncoder());
    SmartDashboard.putNumber("Tilt Error", RobotContainer.shooterSubsystem.getTiltError() );
    SmartDashboard.putNumber("Tilt Angle", RobotContainer.shooterSubsystem.getTiltAngle() );
    SmartDashboard.putNumber("Shooter Power Adjustment", RobotContainer.shooterSubsystem.shooterWheelPowerAdjustment() );
    SmartDashboard.putNumber("Tilt ZT", RobotContainer.shooterSubsystem.getTiltZT() );
  }

  public void ballIntakeCalibrationValues() {
    SmartDashboard.putNumber("Intake Power ", (RobotContainer.turnStick.getRawAxis(3)+1)/2.0);
    SmartDashboard.putNumber("Shooter Wheel Intake Power ", (RobotContainer.turnStick.getRawAxis(3)+1)/2.0);
  }

  public void updateShooterSolutionValues() {
    SmartDashboard.putNumber("Shooter Angle", RobotContainer.shooterSubsystem.getShooterAnglePID() );
    SmartDashboard.putNumber("Shooter Power", RobotContainer.shooterSubsystem.getShooterWheelPID() );
    SmartDashboard.putNumber("Shooter Goal", RobotContainer.shooterSubsystem.getGoalSelection());
    SmartDashboard.putNumber("Shooter Solution Angle", RobotContainer.shooterSubsystem.getShootingSolution()[0]);
    SmartDashboard.putNumber("Shooter Solution Power", RobotContainer.shooterSubsystem.getShootingSolution()[1]);
    SmartDashboard.putNumber("Shooter Attempted Distance", RobotContainer.shooterSubsystem.getAttemptedDistanceSelection());
  }

  public void updateClimberValues() {
    SmartDashboard.putNumber("Climber0 Encoder", RobotContainer.climberSubsystem.getEncoder(0));
    SmartDashboard.putNumber("Climber0 Error", RobotContainer.climberSubsystem.getEncoderError(0));
    SmartDashboard.putNumber("Climber1 Encoder", RobotContainer.climberSubsystem.getEncoder(1));
    SmartDashboard.putNumber("Climber1 Error", RobotContainer.climberSubsystem.getEncoderError(1));
    //SmartDashboard.putNumber("Climber0 FL", RobotContainer.climberSubsystem.climberMotorControllers[1].isFwdLimitSwitchClosed());
    //SmartDashboard.putNumber("Climber0 RL", RobotContainer.climberSubsystem.climberMotorControllers[1].isRevLimitSwitchClosed());
  
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
    SmartDashboard.putNumber("Limelight ty", RobotContainer.shooterSubsystem.getTargetVerticalOffset());
    SmartDashboard.putNumber("Limelight tx", RobotContainer.shooterSubsystem.getTargetHorizontalOffset());
    // SmartDashboard.putNumber("Limelight mean", DescriptiveMath.trimmean(RobotContainer.targetingCamera.getSamples(), 10));
    SmartDashboard.putBoolean("Limelight target detected", RobotContainer.shooterSubsystem.isTargetDetected());
    SmartDashboard.putNumber("Limelight distance", RobotContainer.shooterSubsystem.getTargetDistance());
    SmartDashboard.putBoolean("L7", RobotContainer.shooterSubsystem.isTargetDistance(7));
    SmartDashboard.putBoolean("L9", RobotContainer.shooterSubsystem.isTargetDistance(9));
    SmartDashboard.putBoolean("L14", RobotContainer.shooterSubsystem.isTargetDistance(14));
    SmartDashboard.putBoolean("LBE", RobotContainer.shooterSubsystem.isBullsEye());
    SmartDashboard.putNumber("Limelight RAW distance", RobotContainer.shooterSubsystem.getTargetDistanceRaw());
 
  }
  
  public void updateDIOValues(){
    SmartDashboard.putBoolean("DIO Switch", RobotContainer.dioSubsystem.getSwitchStatus());
  }

  public void updateColorSensorValues() {
    SmartDashboard.putNumber("Shooter Color Sensor Proximity", RobotContainer.colorSensorSubsystem.getObjectProximityShooter());
    SmartDashboard.putString("Shooter Color Detected", RobotContainer.colorSensorSubsystem.getSeenColorShooter());
    SmartDashboard.putBoolean("Shooter Red Ball Detected", RobotContainer.colorSensorSubsystem.isBallRedShooter());
    SmartDashboard.putBoolean("Shooter Blue Ball Detected", RobotContainer.colorSensorSubsystem.isBallBlueShooter());

    if (RobotProperties.isHopperSensor) {
      SmartDashboard.putNumber("Hopper Color Sensor Proximity", RobotContainer.colorSensorSubsystem.getObjectProximityHopper());
      SmartDashboard.putString("Hopper Color Detected", RobotContainer.colorSensorSubsystem.getSeenColorHopper());
      SmartDashboard.putBoolean("Hopper Red Ball Detected", RobotContainer.colorSensorSubsystem.isBallRedHopper());
      SmartDashboard.putBoolean("Hopper Blue Ball Detected", RobotContainer.colorSensorSubsystem.isBallBlueHopper());
    }
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

    updateDIOValues();

    if (Constants.RobotProperties.isIMU) {
      updateIMUValues();
    }

    updateDriveSubsystemTelemetry();

    if (Constants.RobotProperties.isShooter) {
       updateShooterValues();
    }

    updateShooterSolutionValues();

    if (Constants.RobotProperties.isClimber) {
      // updateClimberValues();
    }

    if (Constants.RobotProperties.isTestMotor) {
      updateTestMotorValues();
    }

    if (Constants.RobotProperties.isPotentiometer) {
      updatePotentiometerValues();
    }

    if (Constants.RobotProperties.isColorSensor && (counter%MAXCOUNT==0)) {
      updateColorSensorValues();
      // TODO: decide on the Dashboard telemetry for the color sensors
    }

    if (Constants.RobotProperties.isCANdle && (counter%MAXCOUNT==0)) {
      // ballColorChange();
      // TODO: program CANdle telemetry for the ball color change
    }

    if (RobotProperties.driveInterface == DriveInterface.ONESTICK) {
      updateOUValues();
    }

    /** Telemetry for Intake speed calibration */
   // if ( RobotContainer.bbl.getRawButton(Constants.OIC2022TEST.BallIntakeCalibrationSwitch)) {
    //  ballIntakeCalibrationValues();
    //}

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

    // Telemetry counter
    counter++;
    if (counter>=MAXCOUNT){
      counter=0;
    }
  }
}
