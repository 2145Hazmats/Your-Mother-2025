// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReefConstants {
  private static double[] reefCenter = {4.488, 4.025}; //NEED THIS VALUE
  private static double reefRadius = Units.inchesToMeters(32.15); // INCHES

  private static double distanceOffset = 1;
  private static double leftOffset = 1;
  private static double rightOffset = -1;

  public final class ReefMathConstants { // RUN IN ROBOT CONTAINER TO TEST //IT WORKS IN DESMOS
    //---------------- Calculate 6 Central Poses Around Reef ---------------
    public static double[] ABReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(180)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(180)), 0};
    public static double[] CDReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(240)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(240)), 240};
    public static double[] EFReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(300)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(300)), 300};
    public static double[] GHReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(0)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(0)), 180};
    public static double[] IJReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(60)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(60)), 60};
    public static double[] KLReefCenter = new double[]{reefCenter[0] + (reefRadius + distanceOffset) * Math.cos(Units.degreesToRadians(120)), reefCenter[1] + (reefRadius + distanceOffset) * Math.sin(Units.degreesToRadians(120)), 120};

    //---------------- Calculate 6 Left Offset Poses Around Reef -----------
    public static double[] AReefLeft = new double[]{ABReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(0)), 0};
    public static double[] CReefLeft = new double[]{CDReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(60)), 240};
    public static double[] EReefLeft = new double[]{EFReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(120)), 300};
    public static double[] GReefLeft = new double[]{GHReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(180)), 180};
    public static double[] IReefLeft = new double[]{IJReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(240)), 60};
    public static double[] KReefLeft = new double[]{KLReefCenter[0] - leftOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] + leftOffset * Math.cos(Units.degreesToRadians(300)), 120};
  
    //---------------- Calculate 6 Right Offset Poses Around Reef -----------
    public static double[] BReefRight = new double[]{ABReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(0)), 0};
    public static double[] DReefRight = new double[]{CDReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(60)), 240};
    public static double[] FReefRight = new double[]{EFReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(120)), 300};
    public static double[] HReefRight = new double[]{GHReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(180)), 180};
    public static double[] JReefRight = new double[]{IJReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(240)), 60};
    public static double[] LReefRight = new double[]{KLReefCenter[0] + rightOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] - rightOffset * Math.cos(Units.degreesToRadians(300)), 120};

    //---------------- Converted 6 Central Poses Around Reef ---------------
    public static Pose2d ABReefPose = new Pose2d(ABReefCenter[0], ABReefCenter[1], new Rotation2d(ABReefCenter[2]));
    public static Pose2d CDReefPose = new Pose2d(CDReefCenter[0], CDReefCenter[1], new Rotation2d(CDReefCenter[2]));
    public static Pose2d EFReefPose = new Pose2d(EFReefCenter[0], EFReefCenter[1], new Rotation2d(EFReefCenter[2]));
    public static Pose2d GHReefPose = new Pose2d(GHReefCenter[0], GHReefCenter[1], new Rotation2d(GHReefCenter[2]));
    public static Pose2d IJReefPose = new Pose2d(IJReefCenter[0], IJReefCenter[1], new Rotation2d(IJReefCenter[2]));
    public static Pose2d KLReefPose = new Pose2d(KLReefCenter[0], KLReefCenter[1], new Rotation2d(KLReefCenter[2]));

    //---------------- Converted 12 Blue Poses Around Reef ------------------
    public static Pose2d AReefPose = new Pose2d(AReefLeft[0], AReefLeft[1], new Rotation2d(AReefLeft[2]));
    public static Pose2d BReefPose = new Pose2d(BReefRight[0], BReefRight[1], new Rotation2d(BReefRight[2]));
    public static Pose2d CReefPose = new Pose2d(CReefLeft[0], CReefLeft[1], new Rotation2d(CReefLeft[2]));
    public static Pose2d DReefPose = new Pose2d(DReefRight[0], DReefRight[1], new Rotation2d(DReefRight[2]));
    public static Pose2d EReefPose = new Pose2d(EReefLeft[0], EReefLeft[1], new Rotation2d(EReefLeft[2]));
    public static Pose2d FReefPose = new Pose2d(FReefRight[0], FReefRight[1], new Rotation2d(FReefRight[2]));
    public static Pose2d GReefPose = new Pose2d(GReefLeft[0], GReefLeft[1], new Rotation2d(GReefLeft[2]));
    public static Pose2d HReefPose = new Pose2d(HReefRight[0], HReefRight[1], new Rotation2d(HReefRight[2]));
    public static Pose2d IReefPose = new Pose2d(IReefLeft[0], IReefLeft[1], new Rotation2d(IReefLeft[2]));
    public static Pose2d JReefPose = new Pose2d(JReefRight[0], JReefRight[1], new Rotation2d(JReefRight[2]));
    public static Pose2d KReefPose = new Pose2d(KReefLeft[0], KReefLeft[1], new Rotation2d(KReefLeft[2]));
    public static Pose2d LReefPose = new Pose2d(LReefRight[0], LReefRight[1], new Rotation2d(LReefRight[2]));
  }

  public static void displayReefMath() {
    //---------------- POST NUMBERS TO SMART DASHBOARD ----------------
    SmartDashboard.putNumberArray("ABReefCenter", ReefMathConstants.ABReefCenter);
    SmartDashboard.putNumberArray("CDReefCenter", ReefMathConstants.CDReefCenter);
    SmartDashboard.putNumberArray("EFReefCenter", ReefMathConstants.EFReefCenter);
    SmartDashboard.putNumberArray("GHReefCenter", ReefMathConstants.GHReefCenter);
    SmartDashboard.putNumberArray("IJReefCenter", ReefMathConstants.IJReefCenter);
    SmartDashboard.putNumberArray("KLReefCenter", ReefMathConstants.KLReefCenter);

    SmartDashboard.putNumberArray("AReefLeft", ReefMathConstants.AReefLeft);
    SmartDashboard.putNumberArray("CReefLeft", ReefMathConstants.CReefLeft);
    SmartDashboard.putNumberArray("EReefLeft", ReefMathConstants.EReefLeft);
    SmartDashboard.putNumberArray("GReefLeft", ReefMathConstants.GReefLeft);
    SmartDashboard.putNumberArray("IReefLeft", ReefMathConstants.IReefLeft);
    SmartDashboard.putNumberArray("KReefLeft", ReefMathConstants.KReefLeft);

    SmartDashboard.putNumberArray("BReefRight", ReefMathConstants.BReefRight);
    SmartDashboard.putNumberArray("DReefRight", ReefMathConstants.DReefRight);
    SmartDashboard.putNumberArray("FReefRight", ReefMathConstants.FReefRight);
    SmartDashboard.putNumberArray("HReefRight", ReefMathConstants.HReefRight);
    SmartDashboard.putNumberArray("JReefRight", ReefMathConstants.JReefRight);
    SmartDashboard.putNumberArray("LReefRight", ReefMathConstants.LReefRight);
  }

  public final class PoseConstants {
    public static final Pose2d REEF_SIDE_POSE_AB = ReefMathConstants.ABReefPose; 
    public static final Pose2d REEF_SIDE_POSE_CD = ReefMathConstants.CDReefPose;
    public static final Pose2d REEF_SIDE_POSE_EF = ReefMathConstants.EFReefPose;
    public static final Pose2d REEF_SIDE_POSE_GH = ReefMathConstants.GHReefPose;
    public static final Pose2d REEF_SIDE_POSE_IJ = ReefMathConstants.IJReefPose;
    public static final Pose2d REEF_SIDE_POSE_KL = ReefMathConstants.KLReefPose;
    // public static final Pose2d REEF_SIDE_POSE_AB = new Pose2d(3, 4, new Rotation2d(0)); 
    // public static final Pose2d REEF_SIDE_POSE_CD = new Pose2d(3.75, 2.75, new Rotation2d(Units.degreesToRadians(240))); 
    // public static final Pose2d REEF_SIDE_POSE_EF = new Pose2d(5.25, 2.75, new Rotation2d(Units.degreesToRadians(300))); 
    // public static final Pose2d REEF_SIDE_POSE_GH = new Pose2d(5.9, 4, new Rotation2d(Units.degreesToRadians(180))); // I THINK THIS IS THE WRONG ROTATION
    // public static final Pose2d REEF_SIDE_POSE_IJ = new Pose2d(5.4, 5.5, new Rotation2d(Units.degreesToRadians(60))); 
    // public static final Pose2d REEF_SIDE_POSE_KL = new Pose2d(3.6, 5.5, new Rotation2d(Units.degreesToRadians(120))); 

    public static final Pose2d[] BLUE_REEF_SIDE_POSES = {
      REEF_SIDE_POSE_AB, REEF_SIDE_POSE_CD, REEF_SIDE_POSE_EF,
      REEF_SIDE_POSE_GH, REEF_SIDE_POSE_IJ, REEF_SIDE_POSE_KL
    };

    public static final double RED_TRANSFORMATION_X = 8.5;

    public static final Pose2d[] RED_REEF_SIDE_POSES = {
      new Pose2d(REEF_SIDE_POSE_GH.getX() + 8.5, REEF_SIDE_POSE_GH.getY(), REEF_SIDE_POSE_GH.getRotation()),
      new Pose2d(REEF_SIDE_POSE_IJ.getX() + 8.5, REEF_SIDE_POSE_IJ.getY(), REEF_SIDE_POSE_IJ.getRotation()),
      new Pose2d(REEF_SIDE_POSE_KL.getX() + 8.5, REEF_SIDE_POSE_KL.getY(), REEF_SIDE_POSE_KL.getRotation()),
      new Pose2d(REEF_SIDE_POSE_AB.getX() + 8.5, REEF_SIDE_POSE_AB.getY(), REEF_SIDE_POSE_AB.getRotation()),
      new Pose2d(REEF_SIDE_POSE_CD.getX() + 8.5, REEF_SIDE_POSE_CD.getY(), REEF_SIDE_POSE_CD.getRotation()),
      new Pose2d(REEF_SIDE_POSE_EF.getX() + 8.5, REEF_SIDE_POSE_EF.getY(), REEF_SIDE_POSE_EF.getRotation())
    };

    public static final Pose2d REEF_POSE_A = ReefMathConstants.AReefPose;
    public static final Pose2d REEF_POSE_B = ReefMathConstants.BReefPose;
    public static final Pose2d REEF_POSE_C = ReefMathConstants.CReefPose;
    public static final Pose2d REEF_POSE_D = ReefMathConstants.DReefPose;
    public static final Pose2d REEF_POSE_E = ReefMathConstants.EReefPose;
    public static final Pose2d REEF_POSE_F = ReefMathConstants.FReefPose;
    public static final Pose2d REEF_POSE_G = ReefMathConstants.GReefPose;
    public static final Pose2d REEF_POSE_H = ReefMathConstants.HReefPose;
    public static final Pose2d REEF_POSE_I = ReefMathConstants.IReefPose;
    public static final Pose2d REEF_POSE_J = ReefMathConstants.JReefPose;
    public static final Pose2d REEF_POSE_K = ReefMathConstants.KReefPose;
    public static final Pose2d REEF_POSE_L = ReefMathConstants.LReefPose;
    // public static final Pose2d REEF_POSE_A = new Pose2d(3.2, 4.1, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_B = new Pose2d(3.2, 3.87, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_C = new Pose2d(3.6, 3.1, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_D = new Pose2d(4.1, 2.87, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_E = new Pose2d(5.1, 2.75, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_F = new Pose2d(5.4, 2.9, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_G = new Pose2d(6, 3.8, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_H = new Pose2d(6, 4.2, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_I = new Pose2d(5.4, 5.15, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_J = new Pose2d(5.1, 5.35, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_K = new Pose2d(4, 5.4, new Rotation2d(0));
    // public static final Pose2d REEF_POSE_L = new Pose2d(3.6, 5.2, new Rotation2d(0));

    public static final Pose2d[] BLUE_REEF_POSES = {
      REEF_POSE_A, REEF_POSE_B, REEF_POSE_C, REEF_POSE_D, REEF_POSE_E, REEF_POSE_F, 
      REEF_POSE_G, REEF_POSE_H, REEF_POSE_I, REEF_POSE_J, REEF_POSE_K, REEF_POSE_L
    };

    public static final Transform2d RED_TRANSFORMATION = new Transform2d(8.5, 0, new Rotation2d(0));
    
    public static final Pose2d[] RED_REEF_POSES = {
      REEF_POSE_G.plus(RED_TRANSFORMATION), REEF_POSE_H.plus(RED_TRANSFORMATION), REEF_POSE_I.plus(RED_TRANSFORMATION),
      REEF_POSE_J.plus(RED_TRANSFORMATION), REEF_POSE_K.plus(RED_TRANSFORMATION), REEF_POSE_L.plus(RED_TRANSFORMATION),
      REEF_POSE_A.plus(RED_TRANSFORMATION), REEF_POSE_B.plus(RED_TRANSFORMATION), REEF_POSE_C.plus(RED_TRANSFORMATION),
      REEF_POSE_D.plus(RED_TRANSFORMATION), REEF_POSE_E.plus(RED_TRANSFORMATION), REEF_POSE_F.plus(RED_TRANSFORMATION)
    };

    public static final double BLUE_CORAL_STATION_DEG = 126;
    public static final double RED_TRANSFORMATION_STATION_X = 15.1;
    public static final double RED_CORAL_STATION_DEG = 54;

    public static final Pose2d CORAL_STATION_LEFT_BLUE_POSE = new Pose2d(1.175, 6.926, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_BLUE_POSE = new Pose2d(1.175, 1.1, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));

    public static final Pose2d CORAL_STATION_LEFT_RED_POSE = new Pose2d(
      CORAL_STATION_RIGHT_BLUE_POSE.getX() + RED_TRANSFORMATION_STATION_X,
      CORAL_STATION_RIGHT_BLUE_POSE.getY(),
      new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG))
    );
    public static final Pose2d CORAL_STATION_RIGHT_RED_POSE = new Pose2d(
      CORAL_STATION_LEFT_BLUE_POSE.getX() + RED_TRANSFORMATION_STATION_X,
      CORAL_STATION_LEFT_BLUE_POSE.getY(),
      new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG))
    );
  }
}
