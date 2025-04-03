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
    public static final double SlowMoSpeed = 0.4;

    // PID  Drive to pose Constants
    public static final double P_X = 2; //2.95
    public static final double P_Y = 2; //2.95
    public static final double I_X = 0;
    public static final double I_Y = 0;
    public static final double D_X = 0.027;
    public static final double D_Y = 0.027;
    public static final double P_DEGREE = 0.2; //0.125
    public static final double D_DEGREE = 0;
    public static final double PID_RAD = 4.2145;

    //public static final double FAKE_PID_MAX_SPEED = 0.30; //0.1
    public static final double STATION_PID_MAX = 0.25;

    public static final double FAKE_PID_P = 0.70; //Faster: 70, Good: 0.50
    public static final double FAKE_PID_FEED_FORWARD = 0.019; //Faster: 0.02, Good: 0.025

    //public static final double PROFILED_PID_MAX_VELOCITY = 0.25;
    //public static final double PROFILED_PID_MAX_ACCELERATION = 0.25;

    public static final double ESCAPE_SPEED = -0.75;
    public static final double ESCAPE_TIME = 5;
  }

  public static class PathPlannerConstants {
    // PID Autobuilder
    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATIONAL_PID = new PIDConstants(5, 0, 0);
    
    // PathPlanner Pathfinding Constants
    public static final double MAX_VELOCITY_MPS = 4.3; //3
    public static final double MAX_ACCELERATION_MPS = 6; //4
    public static final double MAX_ANGULAR_VELOCITY_RAD = Units.degreesToRadians(90); //Units.degreesToRadians(90)
    public static final double MAX_ANGULAR_ACCELERATION_RAD = Units.degreesToRadians(400); //Units.degreesToRadians(400)
    public static final double NOMINAL_VOLTAGE_VOLTS = 11.5;

    public static final double PATHFIND_END_SPEED_MPS = 0.1;
    public static final double PATHFIND_END_SPEED_MPS_STATION = 1.5;

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

    public static final double AMBIGUITY_RATIO_CUTOFF = 0.2;
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
    public static final double ElaphantP = 2; //1
    public static final double ElaphantI = 0;
    public static final double ElaphantD = 0.00;//0.001

    // Elevator Level Heights
    public static final double L1Position = -28.2 ; //-29
    public static final double L2Position = -28.2; //-29
    public static final double L3Position = -48.7; //-50.5
    public static final double L4Position = -48.7; //-82;
    public static final double HomePosition = 0.0;

    //public static final double AlgaeOffGetReadyLow = (-82 + -48.7)/2;
    //public static final double AlgaeOffGetReadyHigh = (L3Position+L2Position)/2;

    public static final double DealgifyPositionLow = -28.8; //-25.8
    public static final double DealgifyPositionHigh = -51.6; //-48.6
    public static final double ALGAE_OFF_LOW_END = DealgifyPositionLow + 5;
    public static final double ALGAE_OFF_HIGH_END = DealgifyPositionHigh + 5;

    public static final double GROUND_INTAKE_HEIGHT = 0;
    public static final double L1GROUND_INTAKE_HEIGHT = -34.2;



    public static final double SAFETY_LEVEL = L2Position;
    //public static final double CANCEL_ALGI = 
    public static final double NEAR_HOME = HomePosition-0.5;
    public static final double NEARER_HOME = HomePosition-0.05;
    public static final double NEAR_ALGAE_LOW = DealgifyPositionLow-0.5; // TODO test this
    public static final double NEAR_ALGAE_HIGH = DealgifyPositionHigh-0.5; // TODO test this


    
    public static final double ElevatorJoystickSpeedNerf = 0.4;
  }

  // Shooter BOXX Constants
  public static class shooterBoxxContants {
    public static final int ShooterMotorId = 39;
    public static final double kShooterMotorNominalVoltageConstant = 10.5;
    public static final int kBoxxCoralSensorChannel = 0;
    public static final int kElevatorCoralSensorChannel = 2;
    public static final double kSpitSpeed = -0.6; // -0.45
    public static final double kSuckSpeed = -0.85;
    public static final double kFinalSpeed = -0.4;
    public static final double kAsFastAsPossibleSped = 5;
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
    public static final double ClimbReadySetpoint = -4.1; //-3.176
    public static final double ClimbLockedInSetpoint = 1; //0 is too low... 2 almost works, but chain touches elevator
  }
  
  //Coral Scoring Constants 
  public static class ErrorConstants {
    public static final double ElevatorError = 1; //the elevator is around +- 0.1
    public static final double DriveTrainElevatorUpError = Units.inchesToMeters(24);
    public static final double DriveTrainScoreError = Units.inchesToMeters(1.25); //Used to be: 1.2
    public static final double DriveTrainDegreesError = 8;
    public static final double AlgaeError = 2;
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
   public static final int spitterMotorID = 44; 
   public static final int armMotorID = 27;
   public static final int AlgiSensorID = 1; //others are 0 and 2
   public static final double armP = 0.8; //0.5
   public static final double intakeSpeed = 0.5;
   public static final double outtakeSpeed = -0.5;
   public static final double armspeed = 0.6;

   public static final double HomePosition = 3;
   public static final double FloorPosition = 23.7285;
   public static final double ScoreL1Position = 26.437;
   public static final double GrabPosition = 7.3;
   public static final double DealgifyPosition = 11; //9.5
   public static final double ProcessorPosition = 16;
   public static final double NetPosition = 0;
   public static final double DistanceAwayFromHome = 2;
   public static final Pose2d BLUE_NET_POSES = new Pose2d(0,0,new Rotation2d(0));
   public static final Pose2d RED_NET_POSES =  new Pose2d(0,0,new Rotation2d(0));

   public static final double ALGAE_ARM_ERROR = 2;
  }
  public static class LEDConstants {
  public static final int CANdleID = 89;
    
  }
  
}