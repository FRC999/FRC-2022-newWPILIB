// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DIOSubsystem extends SubsystemBase {
  private DigitalInput shooterLimitSwitch;
  private boolean failToCalibrate = false ;

  /** Creates a new DIOSubsystem. */
  public DIOSubsystem() {
    System.out.println("Setting up DIO "+ 1);

      // initialize the limit switch
      if(shooterLimitSwitch == null) {
        try {
          shooterLimitSwitch = new DigitalInput(1) ;
        } catch (Exception e) { // This should not happen on a RIO, but just in case...
          System.out.println("--- Unable to check Digital Input "+ 1);
          failToCalibrate = true;
        }
      }
      System.out.println("DIO initialized");
  }
  
  public boolean getSwitchStatus(){
    return failToCalibrate || shooterLimitSwitch.get(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
