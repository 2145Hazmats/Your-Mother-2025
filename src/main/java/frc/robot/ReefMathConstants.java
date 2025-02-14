// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReefMathConstants {
  /** Creates a new FieldConstants. */
  private double[] reefCenter = {0, 0}; //NEED THIS VALUE
  private double reefRadius = Units.inchesToMeters(32.15); // INCHES

  private double distanceOffset = 1;
  private double leftOffset = 1;
  private double rightOffset = -1;

  // Central Poses
  private double[] ABReefCenter;
  private double[] CDReefCenter;
  private double[] EFReefCenter;
  private double[] GHReefCenter;
  private double[] IJReefCenter;
  private double[] KLReefCenter;

  // Left Offset Poses
  private double[] ABReefLeft;
  private double[] CDReefLeft;
  private double[] EFReefLeft;
  private double[] GHReefLeft;
  private double[] IJReefLeft;
  private double[] KLReefLeft;

  // Right Offset Poses
  private double[] ABReefRight;
  private double[] CDReefRight;
  private double[] EFReefRight;
  private double[] GHReefRight;
  private double[] IJReefRight;
  private double[] KLReefRight;

  public ReefMathConstants() { // RUN IN ROBOT CONTAINER TO TEST //IT WORKS IN DESMOS

      //---------------- Calculate 6 Central Poses Around Reef ---------------
      ABReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(0)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(0))};
      CDReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(60)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(60))};
      EFReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(120)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(120))};
      GHReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(180)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(180))};
      IJReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(240)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(240))};
      KLReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(300)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(300))};

      //---------------- Calculate 6 Left Offset Poses Around Reef -----------
      ABReefLeft = new double[]{ABReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(0))};
      CDReefLeft = new double[]{CDReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(60))};
      EFReefLeft = new double[]{EFReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(120))};
      GHReefLeft = new double[]{GHReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(180))};
      IJReefLeft = new double[]{IJReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(240))};
      KLReefLeft = new double[]{KLReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(300))};

      //---------------- Calculate 6 Right Offset Poses Around Reef -----------
      ABReefRight = new double[]{ABReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(0))};
      CDReefRight = new double[]{CDReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(60))};
      EFReefRight = new double[]{EFReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(120))};
      GHReefRight = new double[]{GHReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(180))};
      IJReefRight = new double[]{IJReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(240))};
      KLReefRight = new double[]{KLReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(300))};

      //---------------- POST NUMBERS TO SMART DASHBOARD ----------------
      SmartDashboard.putNumberArray("ABReefCenter", ABReefCenter);
      SmartDashboard.putNumberArray("CDReefCenter", CDReefCenter);
      SmartDashboard.putNumberArray("EFReefCenter", EFReefCenter);
      SmartDashboard.putNumberArray("GHReefCenter", GHReefCenter);
      SmartDashboard.putNumberArray("IJReefCenter", IJReefCenter);
      SmartDashboard.putNumberArray("KLReefCenter", KLReefCenter);

      SmartDashboard.putNumberArray("ABReefLeft", ABReefLeft);
      SmartDashboard.putNumberArray("CDReefLeft", CDReefLeft);
      SmartDashboard.putNumberArray("EFReefLeft", EFReefLeft);
      SmartDashboard.putNumberArray("GHReefLeft", GHReefLeft);
      SmartDashboard.putNumberArray("IJReefLeft", IJReefLeft);
      SmartDashboard.putNumberArray("KLReefLeft", KLReefLeft);

      SmartDashboard.putNumberArray("ABReefRight", ABReefRight);
      SmartDashboard.putNumberArray("CDReefRight", CDReefRight);
      SmartDashboard.putNumberArray("EFReefRight", EFReefRight);
      SmartDashboard.putNumberArray("GHReefRight", GHReefRight);
      SmartDashboard.putNumberArray("IJReefRight", IJReefRight);
      SmartDashboard.putNumberArray("KLReefRight", KLReefRight);
  }
  
}
