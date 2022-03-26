// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveInterface;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotProperties;
import frc.robot.commands.AutonomousBackLimelight9ft;
import frc.robot.commands.AutonomousDriveLinear;
import frc.robot.commands.AutonomousPlaceholderCommand;
import frc.robot.commands.AutonomousTrajectoryRioCommand;
import frc.robot.commands.AutonomousTurnToAngle;
import frc.robot.commands.AutonomousTurnToAngleLimelight;
import frc.robot.commands.AutonomousTwoBallLimelight;
import frc.robot.commands.CalibrateShooterArmWithLimitSwitch;
import frc.robot.commands.CommandInterruptor;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.DriveStopCommand;
import frc.robot.commands.DriveFromHubAutonomousCommand;
import frc.robot.commands.FrankenbotExtendSolenoid;
import frc.robot.commands.FrankenbotRetractSolenoid;
import frc.robot.commands.ShooterArmPosition;
import frc.robot.commands.ShooterOneButtonShot;
import frc.robot.commands.ShooterOneButtonShotPreset;
import frc.robot.commands.TESTMotionMagic1motor;
import frc.robot.commands.TESTTenFeetForward;
import frc.robot.commands.TargetAndShootHigh;
import frc.robot.commands.TargetAndShootLow;
import frc.robot.commands.TargetHorizontal;
import frc.robot.commands.TargetVertical;
import frc.robot.commands.TwoBallAuto;
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
  public static Joystick auxStick;
  public static XboxController xboxController;
  public static Joystick bbl; // button box left side
  public static Joystick bbr; // button box right side

  // = new Joystick(OIConstants.driverControllerPort);

  public static NavigationControlSubsystem navigationControlSubsystem;

  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    //Configure SendableChooser to contain autonomous routines.
    AutonomousConfigure();
    
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

  public void AutonomousConfigure() {
    //port autonomous routines as commands
    //sets the default option of the SendableChooser to the simplest autonomous command. (from touching the hub, drive until outside the tarmac zone) 
    autoChooser.setDefaultOption("OneBall Lime", new AutonomousBackLimelight9ft());
    autoChooser.addOption("Two Ball Auto", new AutonomousTwoBallLimelight());
    autoChooser.addOption("OneBall Lime", new AutonomousBackLimelight9ft()); // one-ball limelight-driven command
    //port SendableChooser data to the SmartDashboard
    SmartDashboard.putData(autoChooser);
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
      case THREESTICK: // 3 sticks
        turnStick = new Joystick(OIConstants.turnControllerPort);
        driveStick = new Joystick(OIConstants.driverControllerPort);
        auxStick = new Joystick(OIConstants.auxControllerPort);
        break;
      case ONENEWBB: // 1 stick + button box
        driveStick = new Joystick(OIConstants.driverControllerPort);
        bbl =  new Joystick(OIConstants.bbLeftPort);
        bbr =  new Joystick(OIConstants.bbRightPort);
        break;
      case SPLITNEWBB: // 2-sticks + button box
        turnStick = new Joystick(OIConstants.turnControllerPort);
        driveStick = new Joystick(OIConstants.driverControllerPort);
        bbl =  new Joystick(OIConstants.bbLeftPort);
        bbr =  new Joystick(OIConstants.bbRightPort);
        break;
      case THREENEWBB: // 3-sticks + button box
        turnStick = new Joystick(OIConstants.turnControllerPort);
        driveStick = new Joystick(OIConstants.driverControllerPort);
        auxStick = new Joystick(OIConstants.auxControllerPort);
        bbl =  new Joystick(OIConstants.bbLeftPort);
        bbr =  new Joystick(OIConstants.bbRightPort);
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
       /* new JoystickButton(driveStick, 7)
          .whenPressed(new AutonomousTrajectoryRioCommand("10ftForward.wpilib"))
          .whenReleased(new InstantCommand(driveSubsystem::driveTrainBrakeMode)
          .andThen(new InstantCommand(() -> driveSubsystem.manualDrive(0, 0))); */
        // new JoystickButton(driveStick, 11).whenPressed(new FrankenbotExtendSolenoid());
        // new JoystickButton(driveStick, 12).whenPressed(new FrankenbotRetractSolenoid());

        // new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(shooterTest::motorOn, shooterTest));
        // new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(shooterTest::motorOff, shooterTest));

        break;

      case DEMOBOARD:

        //new JoystickButton(driveStick, 7) // testing trajectory binding
        //  .whileHeld(new AutonomousTrajectoryRioCommand("10ftForward.wpilib"))
        //  .whenReleased(new InstantCommand(driveSubsystem::driveTrainBrakeMode,driveSubsystem)
        //  .andThen(new InstantCommand(() -> driveSubsystem.manualDrive(0, 0))));

        
        
        //new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(shooterSubsystem::calibrateForwardSlow, shooterSubsystem));
        //new JoystickButton(driveStick, 9).whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff, shooterSubsystem));
        //new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(shooterSubsystem::calibrateBackSlow, shooterSubsystem));
        //new JoystickButton(driveStick, 10).whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff, shooterSubsystem));

        //new JoystickButton(driveStick, 7).whenPressed(new TESTShooterArmPosition());
        //new JoystickButton(driveStick, 8).whenPressed(new TESTCalibrateShooterArmWithLimitSwitch());

        //new JoystickButton(driveStick, 9).whenPressed(new InstantCommand(testMotorSubsystem::motorForwardSlow, testMotorSubsystem));
        //new JoystickButton(driveStick, 9).whenReleased(new InstantCommand(testMotorSubsystem::motorOff, testMotorSubsystem));

        //new JoystickButton(driveStick, 10).whenPressed(new InstantCommand(testMotorSubsystem::motorReverseSlow, testMotorSubsystem));
        //new JoystickButton(driveStick, 10).whenReleased(new InstantCommand(testMotorSubsystem::motorOff, testMotorSubsystem));

        // Switch to next shooting goal
        //new JoystickButton(bbl, Constants.OIC2022TEST.ShooterLowerGoalNext)
        //  .whenPressed(new InstantCommand(() -> shooterSubsystem.nextShootingSolution(1)));

        new JoystickButton(driveStick, 11)
          .whenPressed(new  AutonomousDriveLinear(5))
          .whenReleased(new DriveStopCommand());

        //new JoystickButton(driveStick, 12)
        //.whenPressed(new TESTMotionMagic1motor(driveSubsystem.leftDriveTalonFX[0]))
        //.whenReleased(new DriveStopCommand());

        break;

      case C2020:

        new JoystickButton(turnStick, 11)
          .whenPressed(new  AutonomousDriveLinear(2))
          .whenReleased(new DriveStopCommand());

          new JoystickButton(turnStick, 12)
          .whenPressed(new  AutonomousDriveLinear(10))
          .whenReleased(new DriveStopCommand());

        new JoystickButton(turnStick, 9)
          .whenPressed(new  AutonomousTurnToAngle(180))
          .whenReleased(new DriveStopCommand());
        new JoystickButton(turnStick, 10)
          .whenPressed(new  AutonomousTurnToAngle(-180))
          .whenReleased(new DriveStopCommand());

          new JoystickButton(turnStick, 7)
          .whenPressed(new  InstantCommand(driveSubsystem::zeroDriveEncoders, driveSubsystem));

        break;

      case C2022:

        // Test routine
        // TODO: Replace with command sequences for the competition

        // *****************
        // *** DRIVESTICK ***
        // *****************

        new JoystickButton(driveStick, 8)
        .whenPressed(
          new InstantCommand(RobotContainer.intakeSubsystem::rotateIntakeForward,RobotContainer.intakeSubsystem)
          .andThen(new InstantCommand(() -> RobotContainer.shooterSubsystem.startShooterWheelMotorReverse(-0.5),RobotContainer.shooterSubsystem))
          .andThen(new AutonomousDriveLinear(4))
          .andThen(new InstantCommand(RobotContainer.intakeSubsystem::stopIntakeMotor,RobotContainer.intakeSubsystem))
          .andThen(new InstantCommand(RobotContainer.shooterSubsystem::stopShooterWheelMotor,RobotContainer.shooterSubsystem))

        )
        .whenReleased(new DriveStopCommand());

        

        new JoystickButton(driveStick, 7)
        .whenPressed(new  AutonomousTurnToAngleLimelight()
        .andThen(new WaitCommand(0.25))
        .andThen(new AutonomousTurnToAngleLimelight()))
        ;

        new JoystickButton(driveStick, 9)
        .whenPressed(new  AutonomousDriveLinear(2))
        .whenReleased(new DriveStopCommand());

        new JoystickButton(driveStick, 10)
        .whenPressed(new  AutonomousDriveLinear(10))
        .whenReleased(new DriveStopCommand());


        // one-button reverse
        JoystickButton intakeShooterReverse = new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeShooterReverseButton) ;
        intakeShooterReverse
          .whileHeld(   // Rotate intake and shooter wheels to get the ball IN
            new InstantCommand(intakeSubsystem::rotateIntakeReverse,intakeSubsystem)
            .alongWith(new InstantCommand(shooterSubsystem::startShooterWheelMotor,shooterSubsystem)) )
          .whenReleased( // Stop intake and shooter wheel
            new InstantCommand(intakeSubsystem::stopIntakeMotor,intakeSubsystem)
            .alongWith(new InstantCommand(shooterSubsystem::stopShooterWheelMotor,shooterSubsystem)) 
          );

        // Shooter arm calibration
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterArmCalibrateButton)
          .whenPressed(new CalibrateShooterArmWithLimitSwitch());

        // Shooter arm to 45 +- manual adjustment via Z-tail
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterArmAngleButton)
          .whenPressed(new ShooterArmPosition());

        // climber PID test - left arm
        //new JoystickButton(driveStick, 7)
        //  .whenPressed(new InstantCommand(() -> climberSubsystem.extentClimberArm(0)))
        //  .whenReleased(new InstantCommand(() -> climberSubsystem.climberMotorOff(0)))
        //  ;
        // climber PID test - right arm
        //new JoystickButton(driveStick, 8)
        //  .whenPressed(new InstantCommand(() -> climberSubsystem.retractClimberArm(1)))
        //  .whenReleased(new InstantCommand(() -> climberSubsystem.climberMotorOff(1)))
        //  ;


        // Intake UP/DOWN
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeDownButton).whenPressed(new InstantCommand(intakeSubsystem::lowerIntake, intakeSubsystem));
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeUpButton).whenPressed(new InstantCommand(intakeSubsystem::raiseIntake, intakeSubsystem));

        // *****************
        // *** TURNSTICK ***
        // *****************

        new JoystickButton(turnStick, 9)
          .whenPressed(new  AutonomousTurnToAngle(90))
          .whenReleased(new DriveStopCommand());
        new JoystickButton(turnStick, 10)
          .whenPressed(new  AutonomousTurnToAngle(-90))
          .whenReleased(new DriveStopCommand());


        // one-button ball intake
        JoystickButton ballIntoShooterButton = new JoystickButton(turnStick, Constants.OIC2022TEST.BallIntoShooterButton) ;
        ballIntoShooterButton
          .whileHeld(   // Rotate intake and shooter wheels to get the ball IN
            new InstantCommand(intakeSubsystem::rotateIntakeForward,intakeSubsystem)
            .alongWith(new InstantCommand(shooterSubsystem::startShooterWheelMotorReverse,shooterSubsystem)) )
          .whenReleased( // Stop intake and shooter wheel
            new InstantCommand(intakeSubsystem::stopIntakeMotor,intakeSubsystem)
            .alongWith(new InstantCommand(shooterSubsystem::stopShooterWheelMotor,shooterSubsystem)) 
          );

        // Intake UP/DOWN
        new JoystickButton(turnStick, Constants.OIC2022TEST.IntakeDownButton).whenPressed(new InstantCommand(intakeSubsystem::lowerIntake, intakeSubsystem));
        new JoystickButton(turnStick, Constants.OIC2022TEST.IntakeUpButton).whenPressed(new InstantCommand(intakeSubsystem::raiseIntake, intakeSubsystem));
        //new JoystickButton(turnStick, Constants.OIC2022TEST.TurnstickIntakeDownButton).whenPressed(new InstantCommand(intakeSubsystem::lowerIntake, intakeSubsystem));
        //new JoystickButton(turnStick, Constants.OIC2022TEST.TurnstickIntakeUpButton).whenPressed(new InstantCommand(intakeSubsystem::raiseIntake, intakeSubsystem));

        // *****************
        // ***  AUXSTICK ***
        // *****************

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
        //Trigger ballInShooterDetector = new Trigger(() -> colorSensorSubsystem.isBallInShooter());
        Trigger ballInShooterDetector = new Trigger(() -> true);

        // Shoot with current angle, and manual motor power adjustment via tail
        JoystickButton shooterSemiAutoSequence = new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterSemiAutoSequence) ;
        
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

        // Switch shooting goal: HIGH/LOW, up/down
        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterLowerGoalNext)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.nextShootingSolution(1)));

        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterLowerGoalPrevious)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.previousShootingSolution(1)));

        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterHighGoalNext)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.nextShootingSolution(0)));

        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterHighGoalPrevious)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.previousShootingSolution(0)));

        // Shooter arm zero encoder
        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterArmZeroEncoder)
          .whenPressed(new  InstantCommand(shooterSubsystem::zeroTiltMotorEncoder,shooterSubsystem));
          // Shooter arm slowly forward
        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterArmSlowlyForward)
          .whenPressed(new  InstantCommand(shooterSubsystem::calibrateForwardSlow,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff,shooterSubsystem));
        // Shooter arm slowly back
        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterArmSlowlyBack)
          .whenPressed(new  InstantCommand(shooterSubsystem::calibrateBackSlow,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::tiltMotorOff,shooterSubsystem));

        // Set shooter arm angle to the one in firing solution
        new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterTiltFiringSolution)
          .whenPressed(new InstantCommand(() -> shooterSubsystem.tiltShooterArm( (shooterSubsystem.getShootingSolution())[0])) );

        // Shoot using firing solution power
        JoystickButton shooterExecuteFiringSolution = new JoystickButton(auxStick, Constants.OIC2022TEST.ShooterExecuteFiringSolution) ;
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


        // *****************
        // ***  BBLEFT   ***
        // *****************

        // Command interruptor - interrupt interruptable commands that use motors - in case they get stuck
        new JoystickButton(bbl, Constants.OIC2022TEST.CommandInterruptorSwitch)
        .whenPressed(new CommandInterruptor());

        new JoystickButton(bbl, Constants.OIC2022TEST.Autonomous1ballTestButton)
          .whenPressed(new AutonomousBackLimelight9ft());

        new JoystickButton(bbl, Constants.OIC2022TEST.Autonomous2ballTestButton)
          .whenPressed(new AutonomousTwoBallLimelight());

        // Climber arms together
        // Shooter arm slowly UP
        new JoystickButton(bbl, Constants.OIC2022TEST.ClimberDown)
          .whenPressed(new  InstantCommand(climberSubsystem::calibrateBackSlow,climberSubsystem))
          .whenReleased(new InstantCommand(climberSubsystem::climberMotorOff,climberSubsystem));
        // Shooter arm slowly DOWN
        new JoystickButton(bbl, Constants.OIC2022TEST.ClimberUp)
          .whenPressed(new  InstantCommand(climberSubsystem::calibrateForwardSlow,climberSubsystem))
          .whenReleased(new InstantCommand(climberSubsystem::climberMotorOff,climberSubsystem));


        // Shooter plunger and climber arms lock - same pneumatics
        new JoystickButton(bbl, Constants.OIC2022TEST.ShooterPlungerButton)
          .whenPressed(new InstantCommand(shooterSubsystem::extendPlunger,shooterSubsystem));
          //.whenReleased(new InstantCommand(shooterSubsystem::retractPlunger,shooterSubsystem));

        // *****************
        // ***  BBRIGHT   ***
        // *****************

        JoystickButton climberSafetySwitch = new JoystickButton(auxStick,Constants.OIC2022TEST.ClimberSafetySwitch);

        new JoystickButton(bbr, Constants.OIC2022TEST.ClimberUp0)
          //.and(climberSafetySwitch)
          .whileHeld(new  InstantCommand(() -> climberSubsystem.calibrateForwardSlow(0),climberSubsystem))
          .whenInactive(new InstantCommand(() -> climberSubsystem.climberMotorOff(0),climberSubsystem));
    
        new JoystickButton(bbr, Constants.OIC2022TEST.ClimberDown0)
          //.and(climberSafetySwitch)
          .whileHeld(new  InstantCommand(() -> climberSubsystem.calibrateBackSlow(0),climberSubsystem))
          .whenInactive(new InstantCommand(() -> climberSubsystem.climberMotorOff(0),climberSubsystem));

        new JoystickButton(bbr, Constants.OIC2022TEST.ClimberUp1)
          //.and(climberSafetySwitch)
          .whileHeld(new  InstantCommand(() -> climberSubsystem.calibrateForwardSlow(1),climberSubsystem))
          .whenInactive(new InstantCommand(() -> climberSubsystem.climberMotorOff(1),climberSubsystem));
    
        new JoystickButton(bbr, Constants.OIC2022TEST.ClimberDown1)
          //.and(climberSafetySwitch)
          .whileHeld(new  InstantCommand(() -> climberSubsystem.calibrateBackSlow(1),climberSubsystem))
          .whenInactive(new InstantCommand(() -> climberSubsystem.climberMotorOff(1),climberSubsystem));

        new JoystickButton(bbr, Constants.OIC2022TEST.RetractThirdArm)
          //.and(climberSafetySwitch)
          .whenPressed(new  InstantCommand(climberSubsystem::extendThirdArm,climberSubsystem));
        new JoystickButton(bbr, Constants.OIC2022TEST.ExtendThirdArm)
          .whenPressed(new InstantCommand(climberSubsystem::retractThirdArm,climberSubsystem));

        /*
          new JoystickButton(bbr, Constants.OIC2022TEST.PresetClose)
          .whenActive(
            new InstantCommand(() -> shooterSubsystem.setShootingSolution(6,0),shooterSubsystem)
                .andThen(new InstantCommand(() -> shooterSubsystem.tiltShooterArm( (shooterSubsystem.getShootingSolution())[0])))
                .andThen(new WaitCommand(2))
                .andThen(new InstantCommand(shooterSubsystem::extendPlunger))  // push plunger; no subsystem requirement, so not to stop motors
                .andThen(new WaitCommand(1))  // Wait 1 second)
                .andThen(new InstantCommand(shooterSubsystem::retractPlunger))  // retract plunger
                .andThen(new InstantCommand(shooterSubsystem::stopShooterWheelMotor))  // stop shooter motor
              .deadlineWith (
                  new InstantCommand(() -> shooterSubsystem.startShooterWheelMotor( (shooterSubsystem.getShootingSolution())[1])) // Spin the wheels, continue until the end of the sequence
              ) // end deadlinewith
            );
        */

        new JoystickButton(bbr, Constants.OIC2022TEST.PresetClose)
          .whenPressed(new ShooterOneButtonShotPreset(6,0)) ;       // shoot at 6ft high target

        new JoystickButton(bbr, Constants.OIC2022TEST.PresetMiddle)
          .whenPressed(new ShooterOneButtonShotPreset(9,0)) ;       // shoot at 6ft high target

        new JoystickButton(bbr, Constants.OIC2022TEST.PresetFar)
          .whenPressed(new ShooterOneButtonShotPreset(14,0)) ;       // shoot at 14ft high target

        /*
        // Intake FORWARD test
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeInButton)
          .whenPressed(new InstantCommand(intakeSubsystem::rotateIntakeForward,intakeSubsystem))
          .whenReleased(new InstantCommand(intakeSubsystem::stopIntakeMotor,intakeSubsystem));

        // Intake REVERSE test
        new JoystickButton(driveStick, Constants.OIC2022TEST.IntakeReverseButton)
          .whenPressed(new InstantCommand(intakeSubsystem::rotateIntakeReverse,intakeSubsystem))
          .whenReleased(new InstantCommand(intakeSubsystem::stopIntakeMotor,intakeSubsystem));


        // Shooter wheels test - shoot ball
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterWheelButton)
          .whenPressed(new InstantCommand(shooterSubsystem::startShooterWheelMotor,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::stopShooterWheelMotor,shooterSubsystem));

        // Shooter wheels test - ball intake
        new JoystickButton(driveStick, Constants.OIC2022TEST.ShooterWheelReverseButton)
          .whenPressed(new InstantCommand(shooterSubsystem::startShooterWheelMotorReverse,shooterSubsystem))
          .whenReleased(new InstantCommand(shooterSubsystem::stopShooterWheelMotor,shooterSubsystem));

        

          if (RobotProperties.driveInterface == DriveInterface.SPLITNEWBB) {  // BB will be used for special function testing

            // Test targeting

            new JoystickButton(bbl, Constants.OIC2022TEST.TargetHorizontalButton)
              .whenPressed(new TargetHorizontal());

            new JoystickButton(bbl, Constants.OIC2022TEST.TargetVerticalHighButton)
              .whenPressed(new TargetVertical(0));

            new JoystickButton(bbl, Constants.OIC2022TEST.TargetVerticalLowButton)
              .whenPressed(new TargetVertical(1));

            // Test auto-shoot

            new JoystickButton(bbl, Constants.OIC2022TEST.AutoShootHighButton)
              .whenPressed(new TargetAndShootHigh());

            new JoystickButton(bbl, Constants.OIC2022TEST.AutoShootLowButton)
              .whenPressed(new TargetAndShootLow());

            // Test climber

            new JoystickButton(bbl, Constants.OIC2022TEST.ZeroClimberEncoders)
              .whenPressed(new  InstantCommand(climberSubsystem::zeroClimberMotorEncoders,climberSubsystem));

          }
        */
      
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
    return RobotContainer.autoChooser.getSelected();
  }
}
