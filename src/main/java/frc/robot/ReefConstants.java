// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReefConstants {
  // Field Constants
  private static double[] reefCenter = {4.488, 4.025};
  private static double reefRadius = Units.inchesToMeters(32.75);

  // CONSTANTS YOU CAN EDIT
  private static double centerDistanceOffset = 1;
  private static double playingDistanceOffset = 0.43; //0.52
  private static double kLeftOffset = -0.02; // more negative = closer to right side .0185
  private static double kRightOffset = 0.3912; //DO NOT CHANGE.3412

  // Calibration Stuff For Blue Alliance Only
  private static boolean usePose2D = true;//4.021 // no 5
  private static Pose2d playLeftPoseAB = new Pose2d(3.2075, 4.057323, new Rotation2d(0)); //3.971624 id 0 should be this one
  private static Pose2d playRightPoseAB = new Pose2d(3.2075, 3.70955, new Rotation2d(0)); //3.61076 /9
  
  public final class ReefMathConstants {
    //---------------- Calculate 6 Central Poses Around Reef ---------------
    public static double[] ABReefCenter = new double[]{reefCenter[0] + (reefRadius + centerDistanceOffset) * Math.cos(Units.degreesToRadians(180)), reefCenter[1] + (reefRadius + centerDistanceOffset) * Math.sin(Units.degreesToRadians(180)), 0};
    public static double[] CDReefCenter = new double[]{reefCenter[0] + (reefRadius + centerDistanceOffset) * Math.cos(Units.degreesToRadians(240)), reefCenter[1] + (reefRadius + centerDistanceOffset) * Math.sin(Units.degreesToRadians(240)), Math.PI*1/3};
    public static double[] EFReefCenter = new double[]{reefCenter[0] + (reefRadius + centerDistanceOffset) * Math.cos(Units.degreesToRadians(300)), reefCenter[1] + (reefRadius + centerDistanceOffset) * Math.sin(Units.degreesToRadians(300)), Math.PI*2/3};
    public static double[] GHReefCenter = new double[]{reefCenter[0] + (reefRadius + centerDistanceOffset) * Math.cos(Units.degreesToRadians(0)), reefCenter[1] + (reefRadius + centerDistanceOffset) * Math.sin(Units.degreesToRadians(0)), Math.PI};
    public static double[] IJReefCenter = new double[]{reefCenter[0] + (reefRadius + centerDistanceOffset) * Math.cos(Units.degreesToRadians(60)), reefCenter[1] + (reefRadius + centerDistanceOffset) * Math.sin(Units.degreesToRadians(60)), Math.PI*4/3};
    public static double[] KLReefCenter = new double[]{reefCenter[0] + (reefRadius + centerDistanceOffset) * Math.cos(Units.degreesToRadians(120)), reefCenter[1] + (reefRadius + centerDistanceOffset) * Math.sin(Units.degreesToRadians(120)), Math.PI*5/3};
    
    // calculates new offsets
    public static double newLeftOffset = usePose2D ? playLeftPoseAB.getY() - ABReefCenter[1] : kLeftOffset;
    public static double newRightOffset = usePose2D ? ABReefCenter[1] - playRightPoseAB.getY() : kRightOffset;

    //---------------- Calculate 6 Playing Poses Around Reef ---------------
    private static double[] ABReefPlaying = new double[]{reefCenter[0] + (reefRadius + playingDistanceOffset) * Math.cos(Units.degreesToRadians(180)), reefCenter[1] + (reefRadius + playingDistanceOffset) * Math.sin(Units.degreesToRadians(180)), 0};
    private static double[] CDReefPlaying = new double[]{reefCenter[0] + (reefRadius + playingDistanceOffset) * Math.cos(Units.degreesToRadians(240)), reefCenter[1] + (reefRadius + playingDistanceOffset) * Math.sin(Units.degreesToRadians(240)), Math.PI*1/3};
    private static double[] EFReefPlaying = new double[]{reefCenter[0] + (reefRadius + playingDistanceOffset) * Math.cos(Units.degreesToRadians(300)), reefCenter[1] + (reefRadius + playingDistanceOffset) * Math.sin(Units.degreesToRadians(300)), Math.PI*2/3};
    private static double[] GHReefPlaying = new double[]{reefCenter[0] + (reefRadius + playingDistanceOffset) * Math.cos(Units.degreesToRadians(0)), reefCenter[1] + (reefRadius + playingDistanceOffset) * Math.sin(Units.degreesToRadians(0)), Math.PI};
    private static double[] IJReefPlaying = new double[]{reefCenter[0] + (reefRadius + playingDistanceOffset) * Math.cos(Units.degreesToRadians(60)), reefCenter[1] + (reefRadius + playingDistanceOffset) * Math.sin(Units.degreesToRadians(60)), Math.PI*4/3};
    private static double[] KLReefPlaying = new double[]{reefCenter[0] + (reefRadius + playingDistanceOffset) * Math.cos(Units.degreesToRadians(120)), reefCenter[1] + (reefRadius + playingDistanceOffset) * Math.sin(Units.degreesToRadians(120)), Math.PI*5/3};

    //---------------- Calculate 6 Left Offset Poses Around Reef -----------
    public static double[] AReefLeft = new double[]{ABReefPlaying[0] - newLeftOffset * Math.sin(Units.degreesToRadians(0)), ABReefPlaying[1] + newLeftOffset * Math.cos(Units.degreesToRadians(0)), 0};
    public static double[] CReefLeft = new double[]{CDReefPlaying[0] - newLeftOffset * Math.sin(Units.degreesToRadians(60)), CDReefPlaying[1] + newLeftOffset * Math.cos(Units.degreesToRadians(60)), Math.PI*1/3};
    public static double[] EReefLeft = new double[]{EFReefPlaying[0] - newLeftOffset * Math.sin(Units.degreesToRadians(120)), EFReefPlaying[1] + newLeftOffset * Math.cos(Units.degreesToRadians(120)), Math.PI*2/3};
    public static double[] GReefLeft = new double[]{GHReefPlaying[0] - newLeftOffset * Math.sin(Units.degreesToRadians(180)), GHReefPlaying[1] + newLeftOffset * Math.cos(Units.degreesToRadians(180)), Math.PI};
    public static double[] IReefLeft = new double[]{IJReefPlaying[0] - newLeftOffset * Math.sin(Units.degreesToRadians(240)), IJReefPlaying[1] + newLeftOffset * Math.cos(Units.degreesToRadians(240)), Math.PI*4/3};
    public static double[] KReefLeft = new double[]{KLReefPlaying[0] - newLeftOffset * Math.sin(Units.degreesToRadians(300)), KLReefPlaying[1] + newLeftOffset * Math.cos(Units.degreesToRadians(300)), Math.PI*5/3};
  
    //---------------- Calculate 6 Right Offset Poses Around Reef -----------
    public static double[] BReefRight = new double[]{ABReefPlaying[0] + newRightOffset * Math.sin(Units.degreesToRadians(0)), ABReefPlaying[1] - newRightOffset * Math.cos(Units.degreesToRadians(0)), 0};
    public static double[] DReefRight = new double[]{CDReefPlaying[0] + newRightOffset * Math.sin(Units.degreesToRadians(60)), CDReefPlaying[1] - newRightOffset * Math.cos(Units.degreesToRadians(60)), Math.PI*1/3};
    public static double[] FReefRight = new double[]{EFReefPlaying[0] + newRightOffset * Math.sin(Units.degreesToRadians(120)), EFReefPlaying[1] - newRightOffset * Math.cos(Units.degreesToRadians(120)), Math.PI*2/3};
    public static double[] HReefRight = new double[]{GHReefPlaying[0] + newRightOffset * Math.sin(Units.degreesToRadians(180)), GHReefPlaying[1] - newRightOffset * Math.cos(Units.degreesToRadians(180)), Math.PI};
    public static double[] JReefRight = new double[]{IJReefPlaying[0] + newRightOffset * Math.sin(Units.degreesToRadians(240)), IJReefPlaying[1] - newRightOffset * Math.cos(Units.degreesToRadians(240)), Math.PI*4/3};
    public static double[] LReefRight = new double[]{KLReefPlaying[0] + newRightOffset * Math.sin(Units.degreesToRadians(300)), KLReefPlaying[1] - newRightOffset * Math.cos(Units.degreesToRadians(300)), Math.PI*5/3};

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

  //---------------- POST NUMBERS TO SMART DASHBOARD ----------------
  public static void displayReefMath() {
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

    SmartDashboard.putNumber("newRightOffset", ReefMathConstants.newRightOffset);
    SmartDashboard.putNumber("newLeftOffset", ReefMathConstants.newLeftOffset);
  }

  public final class PoseConstants {
    public static final Pose2d REEF_SIDE_POSE_AB = ReefMathConstants.ABReefPose; 
    public static final Pose2d REEF_SIDE_POSE_CD = ReefMathConstants.CDReefPose;
    public static final Pose2d REEF_SIDE_POSE_EF = ReefMathConstants.EFReefPose;
    public static final Pose2d REEF_SIDE_POSE_GH = ReefMathConstants.GHReefPose;
    public static final Pose2d REEF_SIDE_POSE_IJ = ReefMathConstants.IJReefPose;
    public static final Pose2d REEF_SIDE_POSE_KL = ReefMathConstants.KLReefPose;

    public static final Pose2d[] BLUE_REEF_SIDE_POSES = {
      REEF_SIDE_POSE_AB, REEF_SIDE_POSE_CD, REEF_SIDE_POSE_EF,
      REEF_SIDE_POSE_GH, REEF_SIDE_POSE_IJ, REEF_SIDE_POSE_KL
    };

    public static final double RED_TRANSFORMATION_X = 8.5695756472;

    public static final Pose2d[] RED_REEF_SIDE_POSES = {
      new Pose2d(REEF_SIDE_POSE_GH.getX() + RED_TRANSFORMATION_X, REEF_SIDE_POSE_GH.getY(), REEF_SIDE_POSE_GH.getRotation()),
      new Pose2d(REEF_SIDE_POSE_IJ.getX() + RED_TRANSFORMATION_X, REEF_SIDE_POSE_IJ.getY(), REEF_SIDE_POSE_IJ.getRotation()),
      new Pose2d(REEF_SIDE_POSE_KL.getX() + RED_TRANSFORMATION_X, REEF_SIDE_POSE_KL.getY(), REEF_SIDE_POSE_KL.getRotation()),
      new Pose2d(REEF_SIDE_POSE_AB.getX() + RED_TRANSFORMATION_X, REEF_SIDE_POSE_AB.getY(), REEF_SIDE_POSE_AB.getRotation()),
      new Pose2d(REEF_SIDE_POSE_CD.getX() + RED_TRANSFORMATION_X, REEF_SIDE_POSE_CD.getY(), REEF_SIDE_POSE_CD.getRotation()),
      new Pose2d(REEF_SIDE_POSE_EF.getX() + RED_TRANSFORMATION_X, REEF_SIDE_POSE_EF.getY(), REEF_SIDE_POSE_EF.getRotation())
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

    public static final Pose2d[] BLUE_REEF_POSES = {
      REEF_POSE_A, REEF_POSE_B, REEF_POSE_C, REEF_POSE_D, REEF_POSE_E, REEF_POSE_F, 
      REEF_POSE_G, REEF_POSE_H, REEF_POSE_I, REEF_POSE_J, REEF_POSE_K, REEF_POSE_L
    };
    
    public static final Pose2d[] RED_REEF_POSES = {
      new Pose2d(REEF_POSE_G.getX() + RED_TRANSFORMATION_X, REEF_POSE_G.getY(), REEF_POSE_G.getRotation()),
      new Pose2d(REEF_POSE_H.getX() + RED_TRANSFORMATION_X, REEF_POSE_H.getY(), REEF_POSE_H.getRotation()),
      new Pose2d(REEF_POSE_I.getX() + RED_TRANSFORMATION_X, REEF_POSE_I.getY(), REEF_POSE_I.getRotation()),
      new Pose2d(REEF_POSE_J.getX() + RED_TRANSFORMATION_X, REEF_POSE_J.getY(), REEF_POSE_J.getRotation()),
      new Pose2d(REEF_POSE_K.getX() + RED_TRANSFORMATION_X, REEF_POSE_K.getY(), REEF_POSE_K.getRotation()),
      new Pose2d(REEF_POSE_L.getX() + RED_TRANSFORMATION_X, REEF_POSE_L.getY(), REEF_POSE_L.getRotation()),
      new Pose2d(REEF_POSE_A.getX() + RED_TRANSFORMATION_X, REEF_POSE_A.getY(), REEF_POSE_A.getRotation()),
      new Pose2d(REEF_POSE_B.getX() + RED_TRANSFORMATION_X, REEF_POSE_B.getY(), REEF_POSE_B.getRotation()),
      new Pose2d(REEF_POSE_C.getX() + RED_TRANSFORMATION_X, REEF_POSE_C.getY(), REEF_POSE_C.getRotation()),
      new Pose2d(REEF_POSE_D.getX() + RED_TRANSFORMATION_X, REEF_POSE_D.getY(), REEF_POSE_D.getRotation()),
      new Pose2d(REEF_POSE_E.getX() + RED_TRANSFORMATION_X, REEF_POSE_E.getY(), REEF_POSE_E.getRotation()),
      new Pose2d(REEF_POSE_F.getX() + RED_TRANSFORMATION_X, REEF_POSE_F.getY(), REEF_POSE_F.getRotation())
    };

    public static final double BLUE_CORAL_STATION_DEG = 306;
    public static final double RED_CORAL_STATION_DEG = 126;

    public static final Pose2d CORAL_STATION_LEFT_BLUE_PATHFIND_POSE = new Pose2d(1.6, 6.7, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_LEFT_BLUE_POSE = new Pose2d(1.394, 7.165, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_BLUE_PATHFIND_POSE = new Pose2d(1.641, 1.439, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_BLUE_POSE = new Pose2d(1.193, 1.019, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));

    public static final Pose2d CORAL_STATION_LEFT_RED_PATHFIND_POSE = new Pose2d(15.896, 1.411, new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_LEFT_RED_POSE = new Pose2d(16.273, 0.962, new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_RED_PATHFIND_POSE = new Pose2d(15.983, 6.7, new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_RED_POSE = new Pose2d(16.186, 7.136, new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG)));

    public static final Pose2d NET_RED_PATHFIND_POSE = new Pose2d(10.14, 1.861, new Rotation2d(0));
    public static final Pose2d NET_RED_POSE = new Pose2d(9.67, 1.891, new Rotation2d(0));
    public static final Pose2d NET_BLUE_PATHFIND_POSE = new Pose2d(7.3, 6.159, new Rotation2d(Math.PI));
    public static final Pose2d NET_BLUE_POSE = new Pose2d(7.708, 6.159, new Rotation2d(Math.PI));
  }
  
  // Climb Drive Setpoints
  public static final double GENERAL_CLIMB_BLUE_DEG = 180;
  public static final double GENERAL_CLIMB_RED_TRANSFORMATION_X = 0; // TODO: ASK CODY WHAT TO DO!
  public static final double GENERAL_CLIMB_RED_DEG = 0;

  public static final Pose2d CLIMB_BLUE_PROCCESOR_PATHFIND_POSE = new Pose2d(1.7, 6.45, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
  public static final Pose2d CLIMB_BLUE_PROCCESOR_POSE = new Pose2d(1.175, 6.95, new Rotation2d(Units.degreesToRadians(-GENERAL_CLIMB_BLUE_DEG)));
  public static final Pose2d CLIMB_BLUE_CENTER_PATHFIND_POSE = new Pose2d(1.7, 1.55, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
  public static final Pose2d CLIMB_BLUE_CENTER_POSE = new Pose2d(1.175, 1.05, new Rotation2d(Units.degreesToRadians(-GENERAL_CLIMB_BLUE_DEG)));
  public static final Pose2d CLIMB_BLUE_NET_PATHFIND_POSE = new Pose2d(1.7, 6.45, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
  public static final Pose2d CLIMB_BLUE_NET_POSE = new Pose2d(1.175, 6.95, new Rotation2d(Units.degreesToRadians(-GENERAL_CLIMB_BLUE_DEG)));
  
  public static final Pose2d CLIMB_BLUE_PROCCESOR_SIDE = new Pose2d();

}
