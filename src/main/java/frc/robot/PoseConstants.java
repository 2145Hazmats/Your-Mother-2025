package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public final class PoseConstants {
    public static final Pose2d REEF_SIDE_POSE_AB = new Pose2d(3, 4, new Rotation2d(0)); 
    public static final Pose2d REEF_SIDE_POSE_CD = new Pose2d(3.75, 2.75, new Rotation2d(Units.degreesToRadians(240))); 
    public static final Pose2d REEF_SIDE_POSE_EF = new Pose2d(5.25, 2.75, new Rotation2d(Units.degreesToRadians(300))); 
    public static final Pose2d REEF_SIDE_POSE_GH = new Pose2d(5.9, 4, new Rotation2d(Units.degreesToRadians(0))); // I THINK THIS IS THE WRONG ROTATION
    public static final Pose2d REEF_SIDE_POSE_IJ = new Pose2d(5.4, 5.5, new Rotation2d(Units.degreesToRadians(60))); 
    public static final Pose2d REEF_SIDE_POSE_KL = new Pose2d(3.6, 5.5, new Rotation2d(Units.degreesToRadians(120))); 

    public static final Pose2d[] BLUE_REEF_SIDE_POSES = {
      REEF_SIDE_POSE_AB, REEF_SIDE_POSE_CD, REEF_SIDE_POSE_EF,
      REEF_SIDE_POSE_GH, REEF_SIDE_POSE_IJ, REEF_SIDE_POSE_KL
    };

    public static final double RED_TRANSFORMATION_X = 8.5;

    public static final Pose2d[] RED_REEF_SIDE_POSES = {
      new Pose2d(REEF_SIDE_POSE_GH.getX() + 8.5,REEF_SIDE_POSE_GH.getY(), REEF_SIDE_POSE_GH.getRotation()),
      new Pose2d(REEF_SIDE_POSE_IJ.getX() + 8.5,REEF_SIDE_POSE_IJ.getY(), REEF_SIDE_POSE_IJ.getRotation()),
      new Pose2d(REEF_SIDE_POSE_KL.getX() + 8.5,REEF_SIDE_POSE_KL.getY(), REEF_SIDE_POSE_KL.getRotation()),
      new Pose2d(REEF_SIDE_POSE_AB.getX() + 8.5,REEF_SIDE_POSE_AB.getY(), REEF_SIDE_POSE_AB.getRotation()),
      new Pose2d(REEF_SIDE_POSE_CD.getX() + 8.5,REEF_SIDE_POSE_CD.getY(), REEF_SIDE_POSE_CD.getRotation()),
      new Pose2d(REEF_SIDE_POSE_EF.getX() + 8.5,REEF_SIDE_POSE_EF.getY(), REEF_SIDE_POSE_EF.getRotation())
    };

    public static final Pose2d REEF_POSE_A = new Pose2d(3.2, 4.1, new Rotation2d(0));
    public static final Pose2d REEF_POSE_B = new Pose2d(3.2, 3.87, new Rotation2d(0));
    public static final Pose2d REEF_POSE_C = new Pose2d(3.6, 3.1, new Rotation2d(0));
    public static final Pose2d REEF_POSE_D = new Pose2d(4.1, 2.87, new Rotation2d(0));
    public static final Pose2d REEF_POSE_E = new Pose2d(5.1, 2.75, new Rotation2d(0));
    public static final Pose2d REEF_POSE_F = new Pose2d(5.4, 2.9, new Rotation2d(0));
    public static final Pose2d REEF_POSE_G = new Pose2d(6, 3.8, new Rotation2d(0));
    public static final Pose2d REEF_POSE_H = new Pose2d(6, 4.2, new Rotation2d(0));
    public static final Pose2d REEF_POSE_I = new Pose2d(5.4, 5.15, new Rotation2d(0));
    public static final Pose2d REEF_POSE_J = new Pose2d(5.1, 5.35, new Rotation2d(0));
    public static final Pose2d REEF_POSE_K = new Pose2d(4, 5.4, new Rotation2d(0));
    public static final Pose2d REEF_POSE_L = new Pose2d(3.6, 5.2, new Rotation2d(0));

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
