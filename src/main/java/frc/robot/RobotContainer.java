// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveInterface;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotProperties;
import frc.robot.commands.AutonomousPlaceholderCommand;
import frc.robot.commands.AutonomousTrajectoryRioCommand;
import frc.robot.commands.CalibrateShooterArmWithLimitSwitch;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.FrankenbotExtendSolenoid;
import frc.robot.commands.FrankenbotRetractSolenoid;
import frc.robot.commands.ShooterArmPosition;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IMUPassthroughSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavigationControlSubsystem;
import frc.robot.subsystems.NetworkTablesSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PotentiometerSubsystem;
import frc.robot.subsystems.PowerDistributionPanelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
/**
 * Temporary testing
 */
import frc.robot.subsystems.TEMPShooterTestSubsystem;
import frc.robot.subsystems.TESTMotorSubsystem;
import frc.robot.subsystems.TargetingCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final AutonomousPlaceholderCommand placeholderAutoCommand = new AutonomousPlaceholderCommand();

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Only one IMU subsystem should be used
  public static final IMUPassthroughSubsystem imuSubsystem = new IMUPassthroughSubsystem();

  public static final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  // TODO: remove this temporary test when done testing the prototypes
  public static final TEMPShooterTestSubsystem shooterTest = new TEMPShooterTestSubsystem();
  
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  public static final TESTMotorSubsystem testMotorSubsystem = new TESTMotorSubsystem();
  
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public static final PotentiometerSubsystem potentiometerSubsystem = new PotentiometerSubsystem();

  public static final CANdleSubsystem candleSubsystem = new CANdleSubsystem();

  public static final ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();

  // PowerDistributionBoard - used for telemetry information
  public static final PowerDistributionPanelSubsystem pdpSubsystem = new PowerDistributionPanelSubsystem();

  public static NetworkTablesSubsystem networkTablesSubsystem = new NetworkTablesSubsystem();

  // public static TargetingCamera targetingCamera = new TargetingCamera();

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

    configureButtonBindings();

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
        new JoystickButton(driveStick, 7)
          .whenPressed(new AutonomousTrajectoryRioCommand("10ftForward.wpilib"))
          .whenReleased(new InstantCommand(driveSubsystem::driveTrainBrakeMode)
          .andThen(new InstantCommand(() -> driveSubsystem.manualDrive(0, 0))));
        // new JoystickButton(driveStick, 11).whenPressed(new FrankenbotExtendSolenoid());
        // new JoystickButton(driveStick, 12).whenPressed(new FrankenbotRetractSolenoid());

        // new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(shooterTest::motorOn, shooterTest));
        // new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(shooterTest::motorOff, shooterTest));

        break;

      case DEMOBOARD:

        new JoystickButton(driveStick, 7) // testing trajectory binding
          .whileHeld(new AutonomousTrajectoryRioCommand("10ftForward.wpilib"))
          .whenReleased(new InstantCommand(driveSubsystem::driveTrainBrakeMode,driveSubsystem)
          .andThen(new InstantCommand(() -> driveSubsystem.manualDrive(0, 0))));
        
        //new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(shooterSubsystem::calibrateForwardSlow, shooterSubsystem));
        //new JoystickButton(driveStick, 9).whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff, shooterSubsystem));
        //new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(shooterSubsystem::calibrateBackSlow, shooterSubsystem));
        //new JoystickButton(driveStick, 10).whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff, shooterSubsystem));

        //new JoystickButton(driveStick, 7).whenPressed(new TESTShooterArmPosition());
        //new JoystickButton(driveStick, 8).whenPressed(new TESTCalibrateShooterArmWithLimitSwitch());

        new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(testMotorSubsystem::motorForwardSlow, testMotorSubsystem));
        new JoystickButton(driveStick, 9).whenReleased(new InstantCommand(testMotorSubsystem::motorOff, testMotorSubsystem));

        new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(testMotorSubsystem::motorReverseSlow, testMotorSubsystem));
        new JoystickButton(driveStick, 10).whenReleased(new InstantCommand(testMotorSubsystem::motorOff, testMotorSubsystem));

        break;
              
      case C2022:

        // Test routine
        // TODO: Replace with command sequences for the competition

        // Intake UP/DOWN test
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeDownButton).whenPressed(new InstantCommand(intakeSubsystem::lowerIntake, intakeSubsystem));
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeUpButton).whenPressed(new InstantCommand(intakeSubsystem::raiseIntake, intakeSubsystem));

        // Intake FORWARD test
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeInButton)
          .whenPressed(new InstantCommand(intakeSubsystem::rotateIntakeForward,intakeSubsystem))
          .whenReleased(new InstantCommand(intakeSubsystem::stopIntakeMotor,intakeSubsystem));

        // Intake REVERSE test
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeReverseButton)
          .whenPressed(new InstantCommand(intakeSubsystem::rotateIntakeReverse,intakeSubsystem))
          .whenReleased(new InstantCommand(intakeSubsystem::stopIntakeMotor,intakeSubsystem));

        // Climber Calibration
          // Shooter arm slowly forward
        new JoystickButton(turnStick, Constants.OIC2022TEST.ClimberLeftCalibrate)
          .whenPressed(new  InstantCommand(climberSubsystem::calibrateForwardSlow,climberSubsystem))
          .whenReleased(new InstantCommand(climberSubsystem::climberMotorOff,climberSubsystem));
        // Shooter arm slowly back
        new JoystickButton(turnStick, Constants.OIC2022TEST.ClimberRightCalibrate)
          .whenPressed(new  InstantCommand(climberSubsystem::calibrateBackSlow,climberSubsystem))
          .whenReleased(new InstantCommand(climberSubsystem::climberMotorOff,climberSubsystem));


        // Shooter arm test
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterArmAngleButton)
          .whenPressed(new ShooterArmPosition())
          // .whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff,shooterSubsystem))
          ;
          //.whenReleased(new CalibrateShooterArmWithLimitSwitch());  // TODO: keep the shooter arm UP not down

        // Shooter wheels test
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterWheelButton)
          .whenPressed(new InstantCommand(shooterSubsystem::startShooterWheelMotor,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::stopShooterWheelMotor,shooterSubsystem));

        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterWheelReverseButton)
          .whenPressed(new InstantCommand(shooterSubsystem::startShooterWheelMotorReverse,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::stopShooterWheelMotor,shooterSubsystem));

        // Shooter plunger test
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterPlungerButton)
          .whenPressed(new InstantCommand(shooterSubsystem::extendPlunger,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::retractPlunger,shooterSubsystem));

        // Shooter arm calibration
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterArmCalibrateButton)
          .whenPressed(new CalibrateShooterArmWithLimitSwitch());

        // Shooter arm zero encoder
        new JoystickButton(turnStick, Constants.OIC2022TEST.ShooterArmZeroEncoder)
          .whenPressed(new  InstantCommand(shooterSubsystem::zeroTiltMotorEncoder,shooterSubsystem));
          // Shooter arm slowly forward
        new JoystickButton(turnStick, Constants.OIC2022TEST.ShooterArmSlowlyForward)
          .whenPressed(new  InstantCommand(shooterSubsystem::calibrateForwardSlow,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff,shooterSubsystem));
        // Shooter arm slowly back
        new JoystickButton(turnStick, Constants.OIC2022TEST.ShooterArmSlowlyBack)
          .whenPressed(new  InstantCommand(shooterSubsystem::calibrateBackSlow,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff,shooterSubsystem));

        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterLowerGoalNext)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.nextShootingSolution(1)));

        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterLowerGoalPrevious)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.previousShootingSolution(1)));

        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterHighGoalNext)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.nextShootingSolution(0)));

        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterHighGoalPrevious)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.previousShootingSolution(0)));

        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterTiltFiringSolution)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.tiltShooterArm( (shooterSubsystem.getShootingSolution())[0])) );
        
        JoystickButton shooterExecuteFiringSolution = new JoystickButton(turnStick, Constants.OIC2022TEST.ShooterExecuteFiringSolution) ;

        shooterExecuteFiringSolution
          .whenActive(  
              new WaitCommand(1)  // Wait 1 second
                            .andThen(new InstantCommand(shooterSubsystem::extendPlunger))  // push plunger; no subsystem requirement, so not to stop motors
                            .andThen(new WaitCommand(1))  // Wait 1 second)
               .andThen(new InstantCommand(shooterSubsystem::retractPlunger))  // retract plunger
               .andThen(new InstantCommand(shooterSubsystem::stopShooterWheelMotor))  // stop shooter motor
            .deadlineWith (
              new InstantCommand(() -> shooterSubsystem.startShooterWheelMotor( (shooterSubsystem.getShootingSolution())[1])) // Spin the wheels, continue until the end of the sequence
              ) // end deadlinewith
          ); // end whenactive
         
        /**
         * Shooter Semi-auto sequence
         * 
         * Spin the wheels
         *  in parallel:
         *    1. Wait 1 second
         *    2. Push plunger
         *    3. Wait 1 second
         * Then
         * Retract plunger
         * Stop the wheels
         * 
        */

        Trigger ballInShooterDetector = new Trigger(() -> colorSensorSubsystem.isBallInShooter());
        //Trigger ballInShooterDetector = new Trigger(() -> true);

        JoystickButton shooterSemiAutoSequence = new JoystickButton(turnStick, Constants.OIC2022TEST.ShooterSemiAutoSequence) ;
        
        // TODO: Make sure that PID on a shooter arm is not cancelled by this

        shooterSemiAutoSequence
          .and(ballInShooterDetector)
          .whenActive(  
              new WaitCommand(1)  // Wait 1 second
                            .andThen(new InstantCommand(shooterSubsystem::extendPlunger))  // push plunger; no subsystem requirement, so not to stop motors
                            .andThen(new WaitCommand(1))  // Wait 1 second)
               .andThen(new InstantCommand(shooterSubsystem::retractPlunger))  // retract plunger
               .andThen(new InstantCommand(shooterSubsystem::stopShooterWheelMotor))  // stop shooter motor
            .deadlineWith (
                new InstantCommand(shooterSubsystem::startShooterWheelMotor,shooterSubsystem) // Spin the wheels, continue until the end of the sequence
              ) // end deadlinewith
          ); // end whenactive
      
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
    return placeholderAutoCommand;
  }
}
