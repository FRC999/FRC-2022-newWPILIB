// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

/**
 * Default settings set here are overwritten in
 * RobotContainer.configureRobotSettings()
 */

public final class Constants {

    public static enum RobotModel {
        FRANKENBOT, DEMOBOARD, C2022
    }

    public static enum DriveInterface {
        ONESTICK, SPLITSTICK, XBOX, XBOXANDSTICK, ONENEWBB, THREESTICK, SPLITNEWBB, THREENEWBB
    }

    public static final class RobotProperties { // configure the type of robot here, such as presence/absence of a
                                                // device, device type etc

        public static boolean robotLogging = true; // if set to TRUE, enable logging to the USB stick via
                                                   // SimpleCSVLogging

        /**
         * Robot Types: FRANKENBOT (2 falcons, pneumatics, Navx) Demo Board (2 falcons,
         * no pneumatics, Pigeon)
         */

        public static RobotModel robotModel = RobotModel.DEMOBOARD; // This setting must be set properly !!! It
                                                                          // controls the rest of the setup

        public static boolean isIMU;
        public static boolean isNaVX;
        public static DriveInterface driveInterface;
        public static boolean isPneumatics;
        public static boolean isShooter;
        public static boolean isClimber;
        public static boolean isTEMPShooterTest;
        public static boolean isPotentiometer;
        public static boolean isColorSensor;
        public static boolean isCANdle;
        public static boolean isIntake;
        public static boolean isTestMotor;
        public static boolean isHopperSensor; // disable hopper color sensor as suspect in RIO lockups

    }

    public static final class RobotDriveChassisConstants { // configure the physical properties unique to the robot
                                                           // here, such as
        // dimensions, wheel diameter etc
        public static int wheelDiameter = 6; // inches
        public static double distanceBetweenWheels = 20; // inches
        public static int encoderUnitsPerShaftRotation = 2048;
        public static double encoderGearReduction = 11.25;
        public static int encoderUnitsPerRobotRotation = 66500;// thats the SUM of the two (this is just a rough
                                                               // guess, and should be measured)
    }

    public static final class TrajectoryDriving { // constants related to the trajectory driving

        public final static double trajectoryRioPidP_Value = 1.43;// * RobotMap.fullMotorOutput /
                                                                  // encoderUnitsPerShaftRotation;
        public final static double trajectoryRioPidI_Value0 = 0.00;// * RobotMap.fullMotorOutput /
                                                                   // encoderUnitsPerShaftRotation;
        public final static double trajectoryRioPidD_Value0 = 0;

        public final static double feedForwardStatic = 0.541;
        public final static double feedForwardVelocity = 0.305;
        public final static double feedForwardAcceleration = 0.0362;

        // We will leave these here for now. However, if we plan to do multiple
        // independents autonomous paths,
        // we will need means to set them in the NavigationControlSubsystem constructor
        // if needed
        // resetPose method would reset a pose to the initial one with coordinates
        // listed here, facing "east"
        public static double startingPoseX = 0.9144;
        // Note that the Slalom path requires a different value
        public static double startingPoseY = 2.286;

    }

    public static final class DriveConstants {

        public static boolean isInvertdGearBox = false;

        public static int[] leftMotorPortID = new int[] { 1 };
        public static int[] rightMotorPortID = new int[] { 2 };

        public static int[] kLeftEncoderPorts = new int[] { 1 };
        public static int[] kRightEncoderPorts = new int[] { 2 };
        public static boolean kLeftEncoderReversed = false;
        public static boolean kRightEncoderReversed = true;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

        // MotionMagic constants section

        // Closed loop constants
        // How long we wait for a configuration change to happen before we give up and
        // report a failure in milliseconds
        public final static int configureTimeoutMs = 30;
        // Full motor output value
        public final static int fullMotorOutput = 1023;
        // How many milliseconds between each closed loop call
        public final static int closedLoopPeriodMs = 1;
        // Motor neutral dead-band, set to the minimum 0.1%
        public final static double NeutralDeadband = 0.001;

        public final static int Izone_0 = 500;
        public final static double PeakOutput_0 = 1;

        /**
         * Talon PID methods often demand slot ID's, so we wil keep this here I do not
         * think we actually need it with Falcons anymore
         */
        public final static int SLOT_0 = 0;

        // Gains for MotionMagic
        public final static double motionMagicPidP_Value = 0.75;// * fullMotorOutput / encoderUnitsPerShaftRotation;
        public final static double motionMagicPidI_Value = 0.005;// * fullMotorOutput / encoderUnitsPerShaftRotation;
        public final static double motionMagicPidD_Value = 0.01;
        public final static double motionMagicPidF_Value = 2;

        public final static int motionMagicCruiseVelocity = 2250 * 3;
        public final static int motionMagicAcceleration = 2250 * 3;
        public final static int motionMagicSmoothing = 3;

        // Deadband values
        public final static double deadbandX = 0.1;
        public final static double deadbandY = 0.1;
        public final static double deadbandZ = 0.1;

        // The difference between the left and right side encoder values when the robot
        // is rotated 180 degrees
        // Allowable error to exit movement methods
        public static int defaultAcceptableError = 250;

        // Make smoother turns - see Cheezy Driving
        public static double turnAdjust = 0.6;

    }
    /**
     * A class containing relevant measurements of the field, ideally for use in autonomous.
     * 
     */
    public static final class FieldConstants {
        //distance from the wall of the hub to the edge of the tarmac at the widest point
        public final static double tarmacWidestPoint = 84.75; 
        //distance from the wall of the hub to the balls located around the tarmac
        public final static double ballFromHub = 116.17;
        //distance from tarmac tape to ball
        public final static double tapeToBall = 40.44;
    }

    public static final class OIConstants {
        public static final int driverControllerPort = 1;
        public static final int turnControllerPort = 0;
        public static final int auxControllerPort = 2;
        public static final int bbLeftPort = 4;
        public static final int bbRightPort = 3;
        public static final int xboxControllerPort = 1;
    }

    // Button assignment for the 2022 competition robot
    // This will probably be unique for that robot
    public static final class OIC2022TEST {

        // drivestick

        public static final int IntakeShooterReverseButton = 1;
        public static final int ShooterArmCalibrateButton = 3;
        public static final int ShooterArmAngleButton = 4;
        public static final int IntakeDownButton = 11;
        public static final int IntakeUpButton = 12;
        // --
        public static final int ShooterArmDownButton = 7;
        public static final int ShooterWheelButton = 8;
        public static final int ShooterWheelReverseButton = 9;

        public static final int IntakeInButton = 3;

        // turnstick

        public static final int BallIntoShooterButton = 1;
        // --
        // Intake Up - 12
        // Intake Down - 11
        // --
        public static final int IntakeReverseButton = 1;

        // aux stick

        public static final int ShooterSemiAutoSequence = 1; // uses current tilt and power adjusted by Z-tail
        public static final int ShooterHighGoalNext = 6;
        public static final int ShooterHighGoalPrevious = 4;
        public static final int ShooterLowerGoalNext = 5;
        public static final int ShooterLowerGoalPrevious = 3;

        public static final int ShooterArmZeroEncoder = 9;
        public static final int ShooterArmSlowlyForward = 7;
        public static final int ShooterArmSlowlyBack = 8;

        public static final int ShooterTiltFiringSolution = 11; // Tilt shooter arm to the angle in firing solution
        public static final int ShooterExecuteFiringSolution = 12; // current angle, power from firing solution

        // buttonbox1 (left)

        public static final int ClimberSafetySwitch = 1;
        public static final int CommandInterruptorSwitch = 1;
        public static final int ClimberUp0 = 6;
        public static final int ClimberDown0 = 7;
        public static final int ClimberDown = 11;
        public static final int ClimberUp = 9;
        public static final int ShooterPlungerButton = 10;
        // --
        public static final int TargetHorizontalButton = 4;
        public static final int TargetVerticalHighButton = 5;
        public static final int TargetVerticalLowButton = 6;
        public static final int AutoShootLowButton = 7;
        public static final int AutoShootHighButton = 8;
        public static final int ZeroClimberEncoders = 10;

        // buttonbox2 (right)

        // Presents - close, medium, far
        public static final int PresetClose = 3;
        public static final int PresetMiddle = 4;
        public static final int PresetFar = 5;

        public static final int ClimberUp1 = 9;
        public static final int ClimberDown1 = 8;
       
    }
    public static final class PigeonIMU {
        // If you have PigeonIMU, this is the ID of the Talon SRX the IMU is connected
        // to
        public static int pigeonIMUId;
    }

    public static final class PneumaticsConstants {

        public static int compressorCANID;

        // index 0 is forward channel
        // index 1 is reverse channel
        public static int[] SolenoidChannel ;
    }
    public static final class PotentiometerConstants {

        public static int PotentiometerPort = 3;

    }
    public static final class ShooterConstants {

        public static int tiltMotorPortID = 4;
        public static int[] shooterWheelMotorPortIDs = new int[2];

        // index 0 = forward
        // index 1 = reverse
        public static int[] shooterSolenoidChannels;

        //public static int encoderUnitsPerShaftRotation = 4096;
        public static int encoderUnitsPerShaftRotation = 4360;


        public static int shooterLimitSwitchDIOPort = 0;    // Limit switch used for shooter arm zeroing encoder

        // Closed loop constants
        // How long we wait for a configuration change to happen before we give up and
        // report a failure in milliseconds
        public final static int configureTimeoutMs = 30;
        // Full motor output value
        public final static int fullMotorOutput = 1023;
        // How many milliseconds between each closed loop call
        public final static int closedLoopPeriodMs = 1;
        // Motor neutral dead-band, set to the minimum 0.1%
        public final static double NeutralDeadband = 0.001;

        /**
         * 2022 Shooter -
         * With motor inverted, the positive power means tilting BACK/UP (negative power - tilting DOWN)
         * SensorPhase ("false" in thise case) is set to track POSITIVE encoder change on UP rotation
         * (so positive power -> positive encoder rotation)
         */

        // Sensor phase - to ensure that sensor is positive when the output is positive
        public final static boolean SensorPhase =  false;
        // Invert shooter motor
        public final static boolean MotorInvert =  true;  

        // Peak Output - forward and reverse*(-1)
        public final static double PeakOutput = 0.3;

        public final static int Izone_0 = 500;
        public final static double PeakOutput_0 = 1;

        // Closed loop PAN PID parameter values 
        // Modified for Closed loop position control
        public final static int PID_TILT = 0;

        // Gains from 2021 - fast and powerfull
        
        /* 
        // Aggressive settings
        public final static double P_TILT = 1.5;
        public final static double I_TILT = 0.0002;
        public final static double D_TILT = 15;
        */

        public final static double P_TILT = 0.8;
        public final static double I_TILT = 0.0002;
        public final static double D_TILT = 0.8;
        

        // Gains from CTR example - slow and steady
        /*
        // Very weak settings
        public final static double P_TILT = 0.15;
        public final static double I_TILT = 0.0;
        public final static double D_TILT = 1;
        */
        // TODO: adjust gains as needed.

        public final static double F_TILT = 0; // set to zero for position closed loop
        // Allowable error to exit movement methods
        public static int tiltDefaultAcceptableError = 1;
        public static int tiltAcceleration = 50;
        public static int tiltCruiseVelocity = 50;
        public final static int tiltSmoothing = 3;

        /**
        * Talon PID methods often demand slot ID's, so we need to do this :(
        */
        public final static int SLOT_0 = 0;

    }

    public static final class ClimberConstants {

        public static int[] climberMotorPortIDs = new int[2];

        // index 0 = forward
        // index 1 = reverse

        public static int encoderUnitsPerShaftRotation = 4096;

        // Closed loop constants
        // How long we wait for a configuration change to happen before we give up and
        // report a failure in milliseconds
        public final static int configureTimeoutMs = 30;
        // Full motor output value
        public final static int fullMotorOutput = 1023;
        // How many milliseconds between each closed loop call
        public final static int closedLoopPeriodMs = 1;
        // Motor neutral dead-band, set to the minimum 0.1%
        public final static double NeutralDeadband = 0.001;

        /**
         * 2022 Shooter -
         * With motor inverted, the positive power means tilting BACK/UP (negative power - tilting DOWN)
         * SensorPhase ("false" in thise case) is set to track POSITIVE encoder change on UP rotation
         * (so positive power -> positive encoder rotation)
         */

        // Sensor phase - to ensure that sensor is positive when the output is positive
        public final static boolean[] SensorPhase =  {true,false};
        // Invert shooter motor
        public final static boolean[] MotorInvert =  {true,true};  

        // Peak Output - forward and reverse*(-1)
        public final static double PeakOutput = 1.0; // peak output for all modes

        public final static int Izone_0 = 500;
        public final static double PeakOutput_0 = 1;

        // Closed loop PID parameter values 
        // Modified for Closed loop position control
        public final static int PID_CLIMB = 0;

        // Gains from 2021 - fast and powerfull
        
        /* 
        // Aggressive settings
        public final static double P_CLIMB = 1.5;
        public final static double I_CLIMB = 0.0002;
        public final static double D_CLIMB = 15;
        */

        public final static double P_CLIMB = 0.8;
        public final static double I_CLIMB = 0.0002;
        public final static double D_CLIMB = 0.8;
        

        // Gains from CTR example - slow and steady
        /*
        // Very weak settings
        public final static double P_CLIMB = 0.15;
        public final static double I_CLIMB = 0.0;
        public final static double D_CLIMB = 1;
        */
        // TODO: adjust gains as needed.

        public final static double F_CLIMB = 0; // set to zero for position closed loop
        // Allowable error to exit movement methods
        public static int climbDefaultAcceptableError = 1;
        public static int climbAcceleration = 50;
        public static int climbCruiseVelocity = 50;
        public final static int climbSmoothing = 3;

        /**
        * Talon PID methods often demand slot ID's, so we need to do this :(
        */
        public final static int SLOT_0 = 0;

    }

    public static final class CANdleConstants {

        public static final int CANdlePort = 6;
        public static final int XBOXPort = 1 ; // to control CANdle
        public static final int MaxBrightnessAngle = 90;
        public static final int MidBrightnessAngle = 180;
        public static final int ZeroBrightnessAngle = 270;

    }

    public static final class IntakeConstants {
        //index 0 = forward
        //index 1 = reverse
        public static int[] intakeSolenoidChannel;
        public static int intakeMotorPort;
        public static final double intakeForwardSpeed = 0.7;
        public static final double intakeReverseSpeed = -0.7;
    }

    public static final class TestHardwareConstants {
        public static int testMotorPort = 4;
        public static int testEncoderPort = 4;
    }

    public static final int TeamNumber = 999;
}
