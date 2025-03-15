// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  
  // Drive Train Constants
  public static class DrivetrainConstants {
    public static final double MAX_ROTATIONS_PER_SECOND = 0.75;
    
    // Controller Nerf
    public static final double SlowMoSpeed = 0.25;

    // PID  Drive to pose Constants
    public static final double P_X = 2.85; //2.95
    public static final double P_Y = 2.85; //2.95
    public static final double I_X = 0;
    public static final double I_Y = 0;
    public static final double D_X = 0.027;
    public static final double D_Y = 0.027;
    public static final double P_DEGREE = 0.125; //0.1
    public static final double D_DEGREE = 0;
    public static final double PID_RAD = 4.2145;

    public static final double PID_MAX = 0.15;
    public static final double STATION_PID_MAX = 0.25;

    // Feed Forward Constants
    public static final double FEEDFORWARD_CONSTANT = 0.0;
    public static final double FEEDFORWARD_CONSTANT_DEGREE = 0; //.12

    //Margin of Error for Scoring
    public static final double MarginOfErrorXY = Units.inchesToMeters(1); // 2 inch
    public static final double MarginOfErrorDeg = 2.5; //5 degree error : Its doubled cuz math
  }

  public static class PathPlannerConstants {
    // PID Autobuilder
    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATIONAL_PID = new PIDConstants(5, 0, 0);
    
    // PathPlanner Pathfinding Constants
    public static final double MAX_VELOCITY_MPS = 3; //5.450 //2.450
    public static final double MAX_ACCELERATION_MPS = 3; //3.6
    public static final double MAX_ANGULAR_VELOCITY_RAD = Units.degreesToRadians(90); //360
    public static final double MAX_ANGULAR_ACCELERATION_RAD = Units.degreesToRadians(400); //862
    public static final double NOMINAL_VOLTAGE_VOLTS = 11.5;

    public static final double PATHFIND_END_SPEED_MPS = 0.1;
  }

  // Camera Constants
  public static class PhotonVisionConstants {
    //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System
    //The Cameras are mounted on the back of the value so all transform signs are flipped (not rotations). + ➔ -
    
    public static final Transform3d ROBOT_TO_CENTRAL_CAMERA =
        new Transform3d(Units.inchesToMeters(12.460005), Units.inchesToMeters(-10.3415), 0, new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(45)));

    public static final Transform3d ROBOT_TO_LEFT_CAMERA =
        new Transform3d(Units.inchesToMeters(12.766455), Units.inchesToMeters(11.158455), 0, new Rotation3d(0, Units.degreesToRadians(10), 0));
        
    public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA =
        new Transform3d(Units.inchesToMeters(-12.85706), Units.inchesToMeters(7.99237), 0, new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(150)));
        
    public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA =
        new Transform3d(Units.inchesToMeters(-12.85706), Units.inchesToMeters(-7.99237), 0, new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(210)));
  }
    
  // Elevator Constants
  public static class elevatorConstants {
    // MotorIDs
    public static final int motorLeaderID = 20;
    public static final int motorFollowerID = 21;

    // PID
    // public static final double PIDBottom = 0;
    // public static final double PIDL2 = 0;
    // public static final double PIDL3 = 0;
    // public static final double PIDL4 = 0;
    public static final double ElaphantP = 1; //0.015
    public static final double ElaphantI = 0;
    public static final double ElaphantD = 0.00;//0.001

    // Elevator Level Heights
    public static final double L1Position = -29; //-19 is old number
    public static final double L2Position = -29;
    public static final double L3Position = -50.5; //-49.1
    public static final double L4Position = -50.5; //-81.15
    public static final double HomePosition = -0.4; //-.6

    public static final double SAFETY_LEVEL = L1Position/1.5;
    public static final double NEAR_HOME = HomePosition-0.5;
    
    public static final double ElevatorJoystickSpeedNerf = 0.4;
  }

  // Shooter BOXX Constants
  public static class shooterBoxxContants {
    public static final int ShooterMotorId = 39;
    public static final double kShooterMotorNominalVoltageConstant = 10.5;
    public static final int kBoxxCoralSensorChannel = 0;
    public static final int kElevatorCoralSensorChannel = 2;
    public static final double kSpitSpeed = -0.45;
    public static final double kSuckSpeed = -0.75;
    public static final double kFinalSpeed = -0.4;
  }

  public static class ClimbContants {
    public static final int ClimbMotorId = 33;
    public static final double climbForwardSpeed = 1;
    public static final double ClimbBackwardSpeed = -.6;
    public static final int ClimbStop = 0;
    public static final int ClimbServoStop = 0;
    public static final double climbLockServoPosition = 0.25;
    public static final double climbUnlockServoPosition = 0;
    public static final double ClimbInLimit = -3.6;
    public static final double ClimbOutLimit = -1;
    public static final double ClimbP = 0.05;
    public static final double ClimbI = 0;
    public static final double ClimbD = 0;
    public static final double ClimbReadySetpoint = -4.1;
    public static final double ClimbLockedInSetpoint = 0;
  }
  
  //Coral Scoring Constants 
  public static class ErrorConstants {
    public static final double ElevatorError = 0.75;
    public static final double DriveTrainElevatorUpError = Units.inchesToMeters(24);
    public static final double DriveTrainScoreError = Units.inchesToMeters(1);
    public static final double DriveTrainDegreesError = 8;
  }

  // Get Coral Constants
  public static class GetCoral {
    public static final double GetCoralError = Units.inchesToMeters(4);
  }

  public static class ControllerConstants {
    public static enum EVERYTHING_ENUM {SCORE, LEFT_SOURCE, RIGHT_SOURCE, NET, PROCESSOR, CLIMB};
  }

  //Algae Constants
  public static class AlgaeConstants {
   public static final int shooterMotorID = 9999; 
   public static final int armMotorID = 9998;
   public static final double armP = 0;
   public static final double intakeSpeed = 0.5;
   public static final double outtakeSpeed = -0.5;

   public static final double HomePosition = 0;
   public static final double FloorPosition = 0;
   public static final double GrabPosition = 0;
   public static final double ProcessorPosition = 0;
   public static final double NetPosition = 0;
  
   public static final Pose2d BLUE_NET_POSES = new Pose2d(0,0,new Rotation2d(0));
   public static final Pose2d RED_NET_POSES =  new Pose2d(0,0,new Rotation2d(0));

   public static final double ALGAE_ARM_ERROR = 2;
  }
    
  
}