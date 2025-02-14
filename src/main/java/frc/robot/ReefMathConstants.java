// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private double[] AReefLeft;
  private double[] CReefLeft;
  private double[] EReefLeft;
  private double[] GReefLeft;
  private double[] IReefLeft;
  private double[] KReefLeft;

  // Right Offset Poses
  private double[] BReefRight;
  private double[] DReefRight;
  private double[] FReefRight;
  private double[] HReefRight;
  private double[] JReefRight;
  private double[] LReefRight;

  //Final Poses
  private Pose2d AReefPose;
  private Pose2d BReefPose;
  private Pose2d CReefPose;
  private Pose2d DReefPose;
  private Pose2d EReefPose;
  private Pose2d FReefPose;
  private Pose2d GReefPose;
  private Pose2d HReefPose;
  private Pose2d IReefPose;
  private Pose2d JReefPose;
  private Pose2d KReefPose;
  private Pose2d LReefPose;



  public ReefMathConstants() { // RUN IN ROBOT CONTAINER TO TEST //IT WORKS IN DESMOS

      //---------------- Calculate 6 Central Poses Around Reef ---------------
      GHReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(0)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(0))};
      IJReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(60)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(60))};
      KLReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(120)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(120))};
      ABReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(180)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(180))};
      CDReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(240)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(240))};
      EFReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(300)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(300))};

      //---------------- Calculate 6 Left Offset Poses Around Reef -----------
      GReefLeft = new double[]{ABReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(0)), 180};
      IReefLeft = new double[]{CDReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(60)), -120};
      KReefLeft = new double[]{EFReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(120)), -60};
      AReefLeft = new double[]{GHReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(180)), 0};
      CReefLeft = new double[]{IJReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(240)), 60};
      EReefLeft = new double[]{KLReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(300)), 120};
    
      //---------------- Calculate 6 Right Offset Poses Around Reef -----------
      HReefRight = new double[]{ABReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(0)), 180};
      JReefRight = new double[]{CDReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(60)), -120};
      LReefRight = new double[]{EFReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(120)), -60};
      BReefRight = new double[]{GHReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(180)), 0};
      DReefRight = new double[]{IJReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(240)), 60};
      FReefRight = new double[]{KLReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(300)), 120};

      //---------------- Calculate 12 Blue Poses Around Reef ------------------
      AReefPose = new Pose2d(AReefLeft[0], AReefLeft[1], new Rotation2d(AReefLeft[2]));
      BReefPose = new Pose2d(BReefRight[0], BReefRight[1], new Rotation2d(BReefRight[2]));
      CReefPose = new Pose2d(CReefLeft[0], CReefLeft[1], new Rotation2d(CReefLeft[2]));
      DReefPose = new Pose2d(DReefRight[0], DReefRight[1], new Rotation2d(DReefRight[2]));
      EReefPose = new Pose2d(EReefLeft[0], EReefLeft[1], new Rotation2d(EReefLeft[2]));
      FReefPose = new Pose2d(FReefRight[0], FReefRight[1], new Rotation2d(FReefRight[2]));
      GReefPose = new Pose2d(GReefLeft[0], GReefLeft[1], new Rotation2d(GReefLeft[2]));
      HReefPose = new Pose2d(HReefRight[0], HReefRight[1], new Rotation2d(HReefRight[2]));
      IReefPose = new Pose2d(IReefLeft[0], IReefLeft[1], new Rotation2d(IReefLeft[2]));
      JReefPose = new Pose2d(JReefRight[0], JReefRight[1], new Rotation2d(JReefRight[2]));
      KReefPose = new Pose2d(KReefLeft[0], KReefLeft[1], new Rotation2d(KReefLeft[2]));
      LReefPose = new Pose2d(LReefRight[0], LReefRight[1], new Rotation2d(LReefRight[2]));
      
      //---------------- POST NUMBERS TO SMART DASHBOARD ----------------
      SmartDashboard.putNumberArray("ABReefCenter", ABReefCenter);
      SmartDashboard.putNumberArray("CDReefCenter", CDReefCenter);
      SmartDashboard.putNumberArray("EFReefCenter", EFReefCenter);
      SmartDashboard.putNumberArray("GHReefCenter", GHReefCenter);
      SmartDashboard.putNumberArray("IJReefCenter", IJReefCenter);
      SmartDashboard.putNumberArray("KLReefCenter", KLReefCenter);

      SmartDashboard.putNumberArray("AReefLeft", AReefLeft);
      SmartDashboard.putNumberArray("CReefLeft", CReefLeft);
      SmartDashboard.putNumberArray("EReefLeft", EReefLeft);
      SmartDashboard.putNumberArray("GReefLeft", GReefLeft);
      SmartDashboard.putNumberArray("IReefLeft", IReefLeft);
      SmartDashboard.putNumberArray("KReefLeft", KReefLeft);

      SmartDashboard.putNumberArray("BReefRight", BReefRight);
      SmartDashboard.putNumberArray("DReefRight", DReefRight);
      SmartDashboard.putNumberArray("FReefRight", FReefRight);
      SmartDashboard.putNumberArray("HReefRight", HReefRight);
      SmartDashboard.putNumberArray("JReefRight", JReefRight);
      SmartDashboard.putNumberArray("LReefRight", LReefRight);

  }
  
}
