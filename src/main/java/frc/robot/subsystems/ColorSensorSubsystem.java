// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotProperties;

public class ColorSensorSubsystem extends SubsystemBase {

  // TODO: make sure the right color sensor is connected to the right I2C port
  private final I2C.Port hopperI2CPort = I2C.Port.kOnboard; 
  private final I2C.Port shooterI2CPort = I2C.Port.kMXP;

  private ColorSensorV3 hopperColorSensor;
  private ColorSensorV3 shooterColorSensor;
  private Color lastSeenColorHopper, lastSeenColorShooter;

  // TODO: Below are constants for color matching; adjust as needed after testing
  // Color match test
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  /**
   * Proximity is measured 0-10cm with a value in 0..2047 range
   * The larger the value, the closer is the object
   */
  private final int HOPPERPROXIMITYTHRESHOLD = 150;  // Threshold when we consider the object present
  private final int SHOOTERPROXIMITYTHRESHOLD = 400;  // Threshold when we consider the object present

  /** Creates a new TEMPColorSensorTestSubsystem. */
  public ColorSensorSubsystem() {
    if (RobotProperties.isColorSensor) {

      // Instantiate and configure hopper color sensor
      hopperColorSensor = new ColorSensorV3(hopperI2CPort);
      if (! hopperColorSensor.isConnected()) {  // cannot determine the presence of the sensor
        // We need color/proximity sensor for the right shooter sequence, so just print a message for now
        System.out.println("*** === ERROR: hopper color sensor is not detected");
      }

      // Instantiate and configure shooter color sensor
      shooterColorSensor = new ColorSensorV3(shooterI2CPort);
      if (! shooterColorSensor.isConnected()) {  // cannot determine the presence of the sensor
        // We need color/proximity sensor for the right shooter sequence, so just print a message for now
        System.out.println("*** === ERROR: shooter color sensor is not detected");
      }
           
      colorMatcher.addColorMatch(kBlueTarget);
      colorMatcher.addColorMatch(kGreenTarget);
      colorMatcher.addColorMatch(kRedTarget);
      colorMatcher.addColorMatch(kYellowTarget);
    }
    System.out.println("ColorSensorSubsystem Initialization complete");
  }

  public String getSeenColor(Color color) {   // It will return most closely matched color as ENUM
    ColorMatchResult match = colorMatcher.matchClosestColor(color);
    String colorString;
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    return colorString;
  }


  /**
   *  Color confidence is 1 - euclidian distance of the two color vectors
   * May consider using it when detecting a ball color
   * @param color
   * @return double
   */
  public double getSeenColorConfidence(Color color) {   // It will return most closely matched color as ENUM
    ColorMatchResult match = colorMatcher.matchClosestColor(color);
    return match.confidence;
  }

  /**
   * Color detection methods are public in order to allow telemetry
   * @return int
   */

   public String getSeenColorHopper() {
    lastSeenColorHopper = hopperColorSensor.getColor();
    return getSeenColor(lastSeenColorHopper);
  }

  public String getSeenColorShooter() {
    lastSeenColorShooter = shooterColorSensor.getColor();
    return getSeenColor(lastSeenColorShooter);
  }

  /**
   * Proximity methods are public in order to allow telemetry
   * @return int
   */

  public int getObjectProximityHopper() {
    return hopperColorSensor.getProximity();
  }

  public int getObjectProximityShooter() {
    return shooterColorSensor.getProximity();
  }

  /**
   * Methods that check the color of the ball
   * @return boolean
   */

  // TODO: consider checking color confidence to augment results

  public boolean isBallRedHopper(){    // Will return true if a ball is red
    lastSeenColorHopper = hopperColorSensor.getColor();
    return  (colorMatcher.matchClosestColor(lastSeenColorHopper).color == kRedTarget) ;
  }

  public boolean isBallRedShooter(){    // Will return true if a ball is red
    lastSeenColorShooter = shooterColorSensor.getColor();
    return  (colorMatcher.matchClosestColor(lastSeenColorShooter).color == kRedTarget) ;
  }

  public boolean isBallBlueHopper(){     // Will return true if a ball is blue
    lastSeenColorHopper = hopperColorSensor.getColor();
    return (colorMatcher.matchClosestColor(lastSeenColorHopper).color == kBlueTarget) ;
  }

  public boolean isBallBlueShooter(){     // Will return true if a ball is blue
    lastSeenColorShooter = shooterColorSensor.getColor();
    return (colorMatcher.matchClosestColor(lastSeenColorShooter).color == kBlueTarget) ;
  }

  /** 
   * Methods that check the presence of the ball - used in intake/hopper/shooter sequences
   * @return boolean
   */

  public boolean isBallInHopper() {
    return hopperColorSensor.getProximity() >= HOPPERPROXIMITYTHRESHOLD ;
  }
  public boolean isBallInShooter() {
    return shooterColorSensor.getProximity() >= SHOOTERPROXIMITYTHRESHOLD ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
