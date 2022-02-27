// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TargetingCamera extends SubsystemBase {

  private static final int MAXSAMPLES = 100;
  private ArrayList <Double> targetSize = new ArrayList <Double>();

  /** Creates a new TargetingCamera. */
  public TargetingCamera() {}

  public double[] getSamples() {
    double r[] = targetSize.stream().mapToDouble(Double::doubleValue).toArray();
    Arrays.sort(r);
    return r;
  }
  @Override
  public void periodic() {

    targetSize.add((RobotContainer.networkTablesSubsystem == null)?
      0 : RobotContainer.networkTablesSubsystem.getDouble("limelight", "tlong", 0));

    if (targetSize.size() > MAXSAMPLES) {
      targetSize.remove(0);
    }

    // This method will be called once per scheduler run
  }
}
