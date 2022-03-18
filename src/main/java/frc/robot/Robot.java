// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Objects;

import javax.tools.DiagnosticCollector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveInterface;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PigeonIMU;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.PotentiometerConstants;
import frc.robot.Constants.RobotDriveChassisConstants;
import frc.robot.Constants.RobotModel;
import frc.robot.Constants.RobotProperties;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TestHardwareConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * Robot on-board logging
   */
  public static final SimpleCSVLogger simpleCSVLogger = new SimpleCSVLogger();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // determine and set robot model based on the DIO jumpers
    // this needs to be done before any other configuration happens
    determineRobotModel();

    // enable loggin if needed
    if (Constants.RobotProperties.robotLogging) {
      simpleCSVLogger.init(new String[] { "Module" }, new String[] { "Message" }); // start the logging; initialize the log file                                                                           // log file on the USB stick
    }

    // Configure Robot Settings based on the constants
    configureRobotSettings();


    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
      //if (! Objects.isNull(m_robotContainer) && ! Objects.isNull(RobotContainer.driveSubsystem) ) { // if DriveSubsystem is created
      //  RobotContainer.driveSubsystem.driveTrainCoastMode();  // leave the drivetrain in COAST at the end of the game
      //}
    //}

    // disable loggin if needed
    if (Constants.RobotProperties.robotLogging) {
      simpleCSVLogger.forceSync();
      simpleCSVLogger.close();
    }

  }

  @Override
  public void disabledPeriodic() {


  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    if (Constants.RobotProperties.isClimber && ! Objects.isNull(m_robotContainer) && ! Objects.isNull(RobotContainer.shooterSubsystem) ) { // if ShooterSubsystem is created
      RobotContainer.shooterSubsystem.retractPlunger();  // engage climber lock - tied to the plunger solenoid
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    if (Constants.RobotProperties.isClimber && ! Objects.isNull(m_robotContainer) && ! Objects.isNull(RobotContainer.shooterSubsystem) ) { // if ShooterSubsystem is created
      RobotContainer.shooterSubsystem.retractPlunger();  // engage climber lock - tied to the plunger solenoid
    }

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // enable loggin if needed
    if (Constants.RobotProperties.robotLogging) {
      simpleCSVLogger.init(new String[] { "Module" }, new String[] { "Message" }); // start the logging; initialize the
                                                                                   // log file on the USB stick
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Update telemetry
    RobotContainer.smartDashboardSubsystem.updateDriveSubsystemTelemetry();
    RobotContainer.smartDashboardSubsystem.updatePDPValues();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /**
   * We're determining the robot model based on the jumpers plugged into the DIO port on the RIO
   * Only one jumper should be installed (though we can use the same method if we get more robots to identify)
   * 
   * The following jumper position values will be used:
   * 
   * No jumper:   C2022
   * 9:           DEMOBOARD
   * 8:           FRANKENBOT
   */
  private void determineRobotModel() {
    int modelNumber = 0;
    final int startPosition = 8;
    for (int inputNumber = startPosition; inputNumber <= 9; inputNumber++) {
      try (DigitalInput input = new DigitalInput(inputNumber)) {

        System.out.println("DIO " + inputNumber + " " + input.get());

        if ( ! input.get()) { // FALSE means the DIO jumper is installed
          modelNumber = inputNumber;
        }
      } catch (Exception e) { // This should not happen on a RIO, but just in case...
        System.out.println("Unable to check Digital Input "+inputNumber);
      }
    }
    switch(modelNumber) {
      case 0:
              Constants.RobotProperties.robotModel = RobotModel.C2022;
              break;
      case 8:
              Constants.RobotProperties.robotModel = RobotModel.FRANKENBOT;
              break;
      case 9:
              Constants.RobotProperties.robotModel = RobotModel.DEMOBOARD;
              break;
      default: // in case something breaks, we assume it's a competiton robot
              Constants.RobotProperties.robotModel = RobotModel.C2022;

    }

    System.out.println("Robot model " + Constants.RobotProperties.robotModel + " detected");

  }

  private void configureRobotSettings() {

    switch (RobotProperties.robotModel) {
      case FRANKENBOT:

        // Subsystem Settings
        RobotProperties.isIMU = true;
        RobotProperties.isNaVX = true;
        RobotProperties.driveInterface = DriveInterface.SPLITSTICK;
        RobotProperties.isPneumatics = false;
        RobotProperties.isTEMPShooterTest = false;

        // Drivetrain settings
        DriveConstants.isInvertdGearBox = false;
        DriveConstants.leftMotorPortID = new int[] { 9 };
        DriveConstants.rightMotorPortID = new int[] { 10 };
        DriveConstants.kLeftEncoderPorts = new int[] { 9 };
        DriveConstants.kRightEncoderPorts = new int[] { 10 };
        DriveConstants.kLeftEncoderReversed = false;
        DriveConstants.kRightEncoderReversed = true;

        RobotDriveChassisConstants.wheelDiameter = 4;
        RobotDriveChassisConstants.encoderUnitsPerShaftRotation = 2048;
        RobotDriveChassisConstants.encoderGearReduction = 6.1;

        // Pneumatics
        PneumaticsConstants.compressorCANID = 0;
        PneumaticsConstants.SolenoidChannel = new int[] { 0, 7 };

        //Robot.simpleCSVLogger.writeData("******  ---- **** Subsystem Configured", "FRANKENBOT");
        break;
      case DEMOBOARD:

        // Subsystem Settings
        RobotProperties.isIMU = true;
        RobotProperties.isNaVX = false;
        RobotProperties.driveInterface = DriveInterface.ONENEWBB;
        RobotProperties.isPneumatics = false;
        RobotProperties.isShooter = false;
        RobotProperties.isPotentiometer = false;
        RobotProperties.isColorSensor = false;
        RobotProperties.isCANdle = true;
        RobotProperties.isTestMotor = false;

        // Drivetrain settings
        DriveConstants.isInvertdGearBox = false;
        DriveConstants.leftMotorPortID = new int[] { 1 };
        DriveConstants.rightMotorPortID = new int[] { 2 };
        DriveConstants.kLeftEncoderPorts = new int[] { 1 };
        DriveConstants.kRightEncoderPorts = new int[] { 2 };
        DriveConstants.kLeftEncoderReversed = false;
        DriveConstants.kRightEncoderReversed = true;

        // IMU
        PigeonIMU.pigeonIMUId = 3;

        // Shooter settings
        ShooterConstants.tiltMotorPortID = 4;
        ShooterConstants.shooterLimitSwitchDIOPort = 1;

        // Test Motor settings
        TestHardwareConstants.testMotorPort = 4;
        TestHardwareConstants.testEncoderPort = 4;

        //potentiometer
        PotentiometerConstants.PotentiometerPort = 3;
        

        Robot.simpleCSVLogger.writeData("Subsystem Configured", "DEMOBOARD");

        break;

      case C2022:

        // Subsystem Settings
        RobotProperties.isIMU = true;
        RobotProperties.isNaVX = false;
        RobotProperties.driveInterface = DriveInterface.THREENEWBB; // TODO: change to SPLITSTICK before comp
        RobotProperties.isPneumatics = true;
        RobotProperties.isShooter = true;
        RobotProperties.isPotentiometer = false;
        RobotProperties.isIntake = true;
        RobotProperties.isColorSensor = true;
        RobotProperties.isClimber = true;
        RobotProperties.isHopperSensor = false;

        // Drivetrain settings
        DriveConstants.isInvertdGearBox = true;
        DriveConstants.leftMotorPortID = new int[] { 3,4 };
        DriveConstants.rightMotorPortID = new int[] { 1,2 };
        DriveConstants.kLeftEncoderPorts = new int[] { 3,4 };
        DriveConstants.kRightEncoderPorts = new int[] { 1,2 };
        DriveConstants.kLeftEncoderReversed = false;
        DriveConstants.kRightEncoderReversed = true;

        // IMU
        PigeonIMU.pigeonIMUId = 5;

        // Intake Settings
        IntakeConstants.intakeSolenoidChannel = new int[] { 2, 3 };
        IntakeConstants.intakeMotorPort = 5;

        // Shooter settings
        ShooterConstants.tiltMotorPortID = 10;
        ShooterConstants.shooterLimitSwitchDIOPort = 1;
        ShooterConstants.shooterWheelMotorPortIDs = new int[] { 11,12 };
        ShooterConstants.shooterSolenoidChannels = new int[] { 1 , 0 };

        // Climber settings
        ClimberConstants.climberMotorPortIDs = new int[] { 13,14 };
        
        Robot.simpleCSVLogger.writeData("Subsystem Configured", "C2022");

        break;

      default:

        // Subsystem Settings
        RobotProperties.isNaVX = true;
        RobotProperties.driveInterface = DriveInterface.SPLITSTICK;
        RobotProperties.isPneumatics = true;
    }

  }

}
