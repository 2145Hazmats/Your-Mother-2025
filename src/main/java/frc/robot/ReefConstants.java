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
  private static double reefRadius = Units.inchesToMeters(32.75); //0.83185 meters

  public static final Pose2d CENTER_OF_THE_FIELD = new Pose2d(8.775, 4.025, new Rotation2d(0));

  // CONSTANTS YOU CAN EDIT
  private static double centerDistanceOffset = 1; //Testing: 1.3
  private static double playingDistanceOffset = 0.42; //Troy: 0.43, Testing: 0.7
  private static double kLeftOffset = -0.02; // more negative = closer to right side .0185
  private static double kRightOffset = 0.3912; //DO NOT CHANGE.3412

  // LeftBlueStationPose = (1.078, 7.09) // top left || blue left

  // CENTER_OF_THE_FIELD = (8.775, 4.025)

  // x offset = 8.775-1.078 = 7.697
  // y offset = 7.09-4.025 = 3.065

  // (8.775+7.697) , (4.25 + 3.065) //top right || red right (16.472, 7.315)
  // (8.775+7.697) , (4.25-3.065) //bottom right || red left (16.472, 1.185)
  // (8.775-7.697) , (4.25-3.065) //bottom left || blue right (1.078,1.185)

  // Calibration Stuff For Blue Alliance Only
  private static boolean usePose2D = true;
  // During Troy Value: 3.21, 4.01 Troy: 3.205, 4.035 Kettering Week1: 3.2075, 4.057323, Botcave: 3.2075, 4.0342
  // PathPlanner: 4.015
  private static Pose2d playLeftPoseAB = new Pose2d(3.21, 4.01, new Rotation2d(0));
  // During Troy Value: 3.21, 3.692 Troy: 3.205, 3.6855 Kettering Week1: 3.2075, 3.70955
  // PathPlanner: 3.685
  private static Pose2d playRightPoseAB = new Pose2d(3.21, 3.692, new Rotation2d(0));
 
  //private static Pose2d LeftBlueStationPose = new Pose2d(1.078, 7.09, new Rotation2d(0)); 
  
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

    //---------------- NEW Calculate 6 Left Offset PATHFIND Poses Around Reef -----------
    public static double[] PATHFINDAReefLeft = new double[]{ABReefCenter[0] - newLeftOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] + newLeftOffset * Math.cos(Units.degreesToRadians(0)), 0};
    public static double[] PATHFINDCReefLeft = new double[]{CDReefCenter[0] - newLeftOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] + newLeftOffset * Math.cos(Units.degreesToRadians(60)), Math.PI*1/3};
    public static double[] PATHFINDEReefLeft = new double[]{EFReefCenter[0] - newLeftOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] + newLeftOffset * Math.cos(Units.degreesToRadians(120)), Math.PI*2/3};
    public static double[] PATHFINDGReefLeft = new double[]{GHReefCenter[0] - newLeftOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] + newLeftOffset * Math.cos(Units.degreesToRadians(180)), Math.PI};
    public static double[] PATHFINDIReefLeft = new double[]{IJReefCenter[0] - newLeftOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] + newLeftOffset * Math.cos(Units.degreesToRadians(240)), Math.PI*4/3};
    public static double[] PATHFINDKReefLeft = new double[]{KLReefCenter[0] - newLeftOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] + newLeftOffset * Math.cos(Units.degreesToRadians(300)), Math.PI*5/3};
  
    //---------------- NEW Calculate 6 Right Offset PATHFIND Poses Around Reef -----------
    public static double[] PATHFINDBReefRight = new double[]{ABReefCenter[0] + newRightOffset * Math.sin(Units.degreesToRadians(0)), ABReefCenter[1] - newRightOffset * Math.cos(Units.degreesToRadians(0)), 0};
    public static double[] PATHFINDDReefRight = new double[]{CDReefCenter[0] + newRightOffset * Math.sin(Units.degreesToRadians(60)), CDReefCenter[1] - newRightOffset * Math.cos(Units.degreesToRadians(60)), Math.PI*1/3};
    public static double[] PATHFINDFReefRight = new double[]{EFReefCenter[0] + newRightOffset * Math.sin(Units.degreesToRadians(120)), EFReefCenter[1] - newRightOffset * Math.cos(Units.degreesToRadians(120)), Math.PI*2/3};
    public static double[] PATHFINDHReefRight = new double[]{GHReefCenter[0] + newRightOffset * Math.sin(Units.degreesToRadians(180)), GHReefCenter[1] - newRightOffset * Math.cos(Units.degreesToRadians(180)), Math.PI};
    public static double[] PATHFINDJReefRight = new double[]{IJReefCenter[0] + newRightOffset * Math.sin(Units.degreesToRadians(240)), IJReefCenter[1] - newRightOffset * Math.cos(Units.degreesToRadians(240)), Math.PI*4/3};
    public static double[] PATHFINDLReefRight = new double[]{KLReefCenter[0] + newRightOffset * Math.sin(Units.degreesToRadians(300)), KLReefCenter[1] - newRightOffset * Math.cos(Units.degreesToRadians(300)), Math.PI*5/3};

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

    //---------------- NEW Converted 12 PATHFIND Poses Around Reef ---------------
    public static Pose2d PATHFINDAReefPose = new Pose2d(PATHFINDAReefLeft[0], PATHFINDAReefLeft[1], new Rotation2d(PATHFINDAReefLeft[2]));
    public static Pose2d PATHFINDBReefPose = new Pose2d(PATHFINDBReefRight[0], PATHFINDBReefRight[1], new Rotation2d(PATHFINDBReefRight[2]));
    public static Pose2d PATHFINDCReefPose = new Pose2d(PATHFINDCReefLeft[0], PATHFINDCReefLeft[1], new Rotation2d(PATHFINDCReefLeft[2]));
    public static Pose2d PATHFINDDReefPose = new Pose2d(PATHFINDDReefRight[0], PATHFINDDReefRight[1], new Rotation2d(PATHFINDDReefRight[2]));
    public static Pose2d PATHFINDEReefPose = new Pose2d(PATHFINDEReefLeft[0], PATHFINDEReefLeft[1], new Rotation2d(PATHFINDEReefLeft[2]));
    public static Pose2d PATHFINDFReefPose = new Pose2d(PATHFINDFReefRight[0], PATHFINDFReefRight[1], new Rotation2d(PATHFINDFReefRight[2]));
    public static Pose2d PATHFINDGReefPose = new Pose2d(PATHFINDGReefLeft[0], PATHFINDGReefLeft[1], new Rotation2d(PATHFINDGReefLeft[2]));
    public static Pose2d PATHFINDHReefPose = new Pose2d(PATHFINDHReefRight[0], PATHFINDHReefRight[1], new Rotation2d(PATHFINDHReefRight[2]));
    public static Pose2d PATHFINDIReefPose = new Pose2d(PATHFINDIReefLeft[0], PATHFINDIReefLeft[1], new Rotation2d(PATHFINDIReefLeft[2]));
    public static Pose2d PATHFINDJReefPose = new Pose2d(PATHFINDJReefRight[0], PATHFINDJReefRight[1], new Rotation2d(PATHFINDJReefRight[2]));
    public static Pose2d PATHFINDKReefPose = new Pose2d(PATHFINDKReefLeft[0], PATHFINDKReefLeft[1], new Rotation2d(PATHFINDKReefLeft[2]));
    public static Pose2d PATHFINDLReefPose = new Pose2d(PATHFINDLReefRight[0], PATHFINDLReefRight[1], new Rotation2d(PATHFINDLReefRight[2]));

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

    //-------------------------------------------------------------------------

    //------------------------- Station Math Time ------------------------------
    //public static Pose2d StationLeftBlue = new Pose2d((CENTER_OF_THE_FIELD), null, null); //x
    public static Pose2d StationRightBlue = new Pose2d(BReefRight[0], BReefRight[1], new Rotation2d(BReefRight[2])); //circl
    public static Pose2d StationLeftRed = new Pose2d(CReefLeft[0], CReefLeft[1], new Rotation2d(CReefLeft[2])); //square
    public static Pose2d StationRightRed = new Pose2d(DReefRight[0], DReefRight[1], new Rotation2d(DReefRight[2])); //triange
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

    // NEW NEW NEW
    public static final Pose2d PATHFIND_REEF_POSE_A = ReefMathConstants.PATHFINDAReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_B = ReefMathConstants.PATHFINDBReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_C = ReefMathConstants.PATHFINDCReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_D = ReefMathConstants.PATHFINDDReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_E = ReefMathConstants.PATHFINDEReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_F = ReefMathConstants.PATHFINDFReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_G = ReefMathConstants.PATHFINDGReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_H = ReefMathConstants.PATHFINDHReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_I = ReefMathConstants.PATHFINDIReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_J = ReefMathConstants.PATHFINDJReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_K = ReefMathConstants.PATHFINDKReefPose;
    public static final Pose2d PATHFIND_REEF_POSE_L = ReefMathConstants.PATHFINDLReefPose;

    // NEW NEW NEW
    public static final Pose2d[] PATHFIND_BLUE_REEF_POSES = {
      PATHFIND_REEF_POSE_A, PATHFIND_REEF_POSE_B, PATHFIND_REEF_POSE_C,
      PATHFIND_REEF_POSE_D, PATHFIND_REEF_POSE_E, PATHFIND_REEF_POSE_F,
      PATHFIND_REEF_POSE_G, PATHFIND_REEF_POSE_H, PATHFIND_REEF_POSE_I,
      PATHFIND_REEF_POSE_J, PATHFIND_REEF_POSE_K, PATHFIND_REEF_POSE_L
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

    // NEW NEW NEW
    public static final Pose2d[] PATHFIND_RED_REEF_POSES = {
      new Pose2d(PATHFIND_REEF_POSE_G.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_G.getY(), PATHFIND_REEF_POSE_G.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_H.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_H.getY(), PATHFIND_REEF_POSE_H.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_I.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_I.getY(), PATHFIND_REEF_POSE_I.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_J.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_J.getY(), PATHFIND_REEF_POSE_J.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_K.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_K.getY(), PATHFIND_REEF_POSE_K.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_L.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_L.getY(), PATHFIND_REEF_POSE_L.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_A.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_A.getY(), PATHFIND_REEF_POSE_A.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_B.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_B.getY(), PATHFIND_REEF_POSE_B.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_C.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_C.getY(), PATHFIND_REEF_POSE_C.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_D.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_D.getY(), PATHFIND_REEF_POSE_D.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_E.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_E.getY(), PATHFIND_REEF_POSE_E.getRotation()),
      new Pose2d(PATHFIND_REEF_POSE_F.getX() + RED_TRANSFORMATION_X, PATHFIND_REEF_POSE_F.getY(), PATHFIND_REEF_POSE_F.getRotation()),
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

    // Izquierdo Stationa y Derecho Stationa

    public static final double BLUE_CORAL_STATION_DEG = 306;
    public static final double RED_CORAL_STATION_DEG = 126;

    // public static final Pose2d CORAL_STATION_LEFT_BLUE_PATHFIND_POSE = new Pose2d(1.6, 6.7, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG)));
    // public static final Pose2d CORAL_STATION_LEFT_BLUE_POSE = new Pose2d(1.058, 7.20, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG))); //1.082, 7.057 //Bot Cave: 1.394, 7.165
    // public static final Pose2d CORAL_STATION_RIGHT_BLUE_PATHFIND_POSE = new Pose2d(1.641, 1.439, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));
    // public static final Pose2d CORAL_STATION_RIGHT_BLUE_POSE = new Pose2d(1.058, 1.075, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));
    
    
    // public static final Pose2d CORAL_STATION_LEFT_RED_PATHFIND_POSE = new Pose2d(15.896, 1.411, new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG)));
    // public static final Pose2d CORAL_STATION_LEFT_RED_POSE = new Pose2d(16.582, 1.275, new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG)));
    // public static final Pose2d CORAL_STATION_RIGHT_RED_PATHFIND_POSE = new Pose2d(15.983, 6.7, new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG)));
    // public static final Pose2d CORAL_STATION_RIGHT_RED_POSE = new Pose2d(16.582, 7.225, new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG)));

    
    public static final Pose2d CORAL_STATION_LEFT_BLUE_PATHFIND_POSE = new Pose2d(1.6, 6.7, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_LEFT_BLUE_POSE = new Pose2d(1.163, 7.142, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG))); //1.082, 7.057 //Bot Cave: 1.394, 7.165
    public static final Pose2d CORAL_STATION_RIGHT_BLUE_PATHFIND_POSE = new Pose2d(1.641, 1.439, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_BLUE_POSE = new Pose2d(0.965, 0.991, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));
    
    
    public static final Pose2d CORAL_STATION_LEFT_RED_PATHFIND_POSE = new Pose2d(15.896, 1.411, new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_LEFT_RED_POSE = new Pose2d(16.435, 0.920, new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_RED_PATHFIND_POSE = new Pose2d(15.983, 6.7, new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG)));
    public static final Pose2d CORAL_STATION_RIGHT_RED_POSE = new Pose2d(16.387, 7.142, new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG)));
    // Net y Processor 
    public static final Pose2d NET_RED_PATHFIND_POSE = new Pose2d(10.14, 1.861, new Rotation2d(0));
    public static final Pose2d NET_RED_POSE = new Pose2d(9.67, 1.891, new Rotation2d(0));
    public static final Pose2d NET_BLUE_PATHFIND_POSE = new Pose2d(7.3, 6.159, new Rotation2d(Math.PI));
    public static final Pose2d NET_BLUE_POSE = new Pose2d(7.708, 6.159, new Rotation2d(Math.PI));

    public static final Pose2d PROCESSOR_RED_PATHFIND_POSE = new Pose2d(11.505, 7.174, new Rotation2d(Math.PI/2));
    public static final Pose2d PROCESSOR_RED_POSE = new Pose2d(11.515, 7.476, new Rotation2d(Math.PI/2));
    public static final Pose2d PROCESSOR_BLUE_PATHFIND_POSE = new Pose2d(5.889, 1.051, new Rotation2d(-Math.PI/2));
    public static final Pose2d PROCESSOR_BLUE_POSE = new Pose2d(5.938, 0.564, new Rotation2d(-Math.PI/2));
  
    // Climb Drive Setpoints
    public static final double GENERAL_CLIMB_BLUE_DEG = -170;
    public static final double GENERAL_CLIMB_RED_DEG = 10;

    public static final Pose2d CLIMB_BLUE_PROCCESOR_PATHFIND_POSE = new Pose2d(7.144, 5.004, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
    public static final Pose2d CLIMB_BLUE_PROCCESOR_POSE =          new Pose2d(8.331, 5.004, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
    public static final Pose2d CLIMB_BLUE_CENTER_PATHFIND_POSE =    new Pose2d(7.144, 6.105, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
    public static final Pose2d CLIMB_BLUE_CENTER_POSE =             new Pose2d(8.331, 6.105, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
    public static final Pose2d CLIMB_BLUE_NET_PATHFIND_POSE =       new Pose2d(7.144, 7.184, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
    public static final Pose2d CLIMB_BLUE_NET_POSE =                new Pose2d(8.331, 7.184, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_BLUE_DEG)));
    
    public static final Pose2d CLIMB_RED_PROCCESOR_PATHFIND_POSE = new Pose2d(10.468, 3.085, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_RED_DEG)));
    public static final Pose2d CLIMB_RED_PROCCESOR_POSE =          new Pose2d(9.226, 3.085, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_RED_DEG)));
    public static final Pose2d CLIMB_RED_CENTER_PATHFIND_POSE =    new Pose2d(10.468, 2.001, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_RED_DEG)));
    public static final Pose2d CLIMB_RED_CENTER_POSE =             new Pose2d(9.226, 2.001, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_RED_DEG)));
    public static final Pose2d CLIMB_RED_NET_PATHFIND_POSE =       new Pose2d(10.468, 0.917, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_RED_DEG)));
    public static final Pose2d CLIMB_RED_NET_POSE =                new Pose2d(9.226, 0.917, new Rotation2d(Units.degreesToRadians(GENERAL_CLIMB_RED_DEG)));

  }
}
