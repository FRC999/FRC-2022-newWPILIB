// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveInterface;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotProperties;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FrankenbotExtendSolenoid;
import frc.robot.commands.FrankenbotRetractSolenoid;
import frc.robot.commands.TESTCalibrateShooterArmWithLimitSwitch;
import frc.robot.commands.TESTShooterArmPosition;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IMUPassthroughSubsystem;
import frc.robot.subsystems.NavigationControlSubsystem;
import frc.robot.subsystems.NetworkTablesSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PotentiometerSubsystem;
import frc.robot.subsystems.PowerDistributionPanelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.TEMPColorSensorTestSubsystem;
/**
 * Temporary testing
 */
import frc.robot.subsystems.TEMPShooterTestSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Only one IMU subsystem should be used
  public static final IMUPassthroughSubsystem imuSubsystem = new IMUPassthroughSubsystem();

  public static final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  // TODO: remove this temporary test when done testing the prototypes
  public static final TEMPShooterTestSubsystem shooterTest = new TEMPShooterTestSubsystem();
  
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public static final PotentiometerSubsystem potentiometerSubsystem = new PotentiometerSubsystem();

  public static final CANdleSubsystem candleSubsystem = new CANdleSubsystem();

  public static final TEMPColorSensorTestSubsystem colorSensorTestSubsystem = new TEMPColorSensorTestSubsystem();

  // PowerDistributionBoard - used for telemetry information
  public static final PowerDistributionPanelSubsystem pdpSubsystem = new PowerDistributionPanelSubsystem();

  public static NetworkTablesSubsystem networkTablesSubsystem = new NetworkTablesSubsystem();

  // Telemetry subsustems must be instantiated last. We report on the other
  // subsystems there
  // Most of the methods in this subsystem are static
  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();

  public static final ShuffleboardSubsystem shuffleboardSubsystem = new ShuffleboardSubsystem();

  // The driver's controller - create variables, but only the ones needed will be
  // initialized
  public static Joystick driveStick;
  public static Joystick turnStick;
  public static XboxController xboxController;

  // = new Joystick(OIConstants.driverControllerPort);

  public static NavigationControlSubsystem navigationControlSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    // Configure the button bindings
    configureDriverInterface();

    //imuSubsystem = new IMUPassthroughSubsystem();
    //pneumaticsSubsystem = new PneumaticsSubsystem();

    configureButtonBindings();

    // Set Driver telemetry
    shuffleboardSubsystem.setDriveSubsystemTelemetry(driveSubsystem);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveSubsystem.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // new RunCommand(() ->
        // driveSubsystem.arcadeDrive(joystick.getLeftY(),
        // joystick.getRightX()), driveSubsystem));

        new DriveManuallyCommand());

    // Don't start kinematics untill we're ready
    if (Constants.RobotProperties.isIMU) {
      navigationControlSubsystem = new NavigationControlSubsystem(driveSubsystem, imuSubsystem);
    }

  }

  /**
   * Use this method to define your controllers depending on the
   * {@link DriveInterface}
   */
  private void configureDriverInterface() {
    switch (RobotProperties.driveInterface) {
      case SPLITSTICK: // add 2 sticks
        turnStick = new Joystick(OIConstants.turnControllerPort);
        driveStick = new Joystick(OIConstants.driverControllerPort);
        break;
      case ONESTICK: // add 1 stick
        driveStick = new Joystick(OIConstants.driverControllerPort);
        break;
      case XBOXANDSTICK: // 1 stick and XBOX controller are created
        driveStick = new Joystick(OIConstants.driverControllerPort);
      case XBOX: // just the XBOX controller
        xboxController = new XboxController(OIConstants.xboxControllerPort);
        break;
      default: // ONESTICK
        turnStick = new Joystick(OIConstants.turnControllerPort);
    }
    System.out.println("Driver interface configured as " + RobotProperties.driveInterface.name());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    switch (RobotProperties.robotModel) {
      case FRANKENBOT:

        new JoystickButton(driveStick, 11).whenPressed(new FrankenbotExtendSolenoid());
        new JoystickButton(driveStick, 12).whenPressed(new FrankenbotRetractSolenoid());

        new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(shooterTest::motorOn1, shooterTest));
        new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(shooterTest::motorOff1, shooterTest));

        break;

      case DEMOBOARD:
        
        new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(shooterSubsystem::calibrateForwardSlow, shooterSubsystem));
        new JoystickButton(driveStick, 9).whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff, shooterSubsystem));
        new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(shooterSubsystem::calibrateBackSlow, shooterSubsystem));
        new JoystickButton(driveStick, 10).whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff, shooterSubsystem));

        new JoystickButton(driveStick, 7).whenPressed(new TESTShooterArmPosition());
        new JoystickButton(driveStick, 8).whenPressed(new TESTCalibrateShooterArmWithLimitSwitch());

        break;

      default:
    }

    Robot.simpleCSVLogger.writeData("ButtonBinding Configured");

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
