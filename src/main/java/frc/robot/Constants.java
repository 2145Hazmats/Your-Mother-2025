// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  
  // Drive Train Constants
  public static class DrivetrainConstants {
    public static final double MAX_ROTATIONS_PER_SECOND = 0.75;
    
    // Controller Nerf
    public static final double SlowMoSpeed = 0.3;

    // PID  Drive to pose Constants
    public static final double P_X = 1.68; //1.7
    public static final double P_Y = 1.68; //1.7
    public static final double I_X = 0; // 0.001
    public static final double I_Y = 0; // 0.001
    public static final double D_X = 1; //0.5
    public static final double D_Y = 1; //0.5
    public static final double P_DEGREE = 0.15;
    public static final double D_DEGREE = 0;
    public static final double PID_RAD = 0;

    // Feed Forward Constants
    //TODO: DISABLE FEEDFORWARD BECAUSE WE ADDED kS ?????
    public static final double FEEDFORWARD_CONSTANT = 0.0;
    public static final double FEEDFORWARD_CONSTANT_DEGREE = 0; //.12
  }

  public static class PathPlannerConstants {
    // PID Autobuilder
    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATIONAL_PID = new PIDConstants(5, 0, 0);
    
    // PathPlanner Pathfinding Constants
    public static final double MAX_VELOCITY_MPS = 2.450; //5.450
    public static final double MAX_ACCELERATION_MPS = 3.6;
    public static final double MAX_ANGULAR_VELOCITY_RAD = Units.degreesToRadians(360);
    public static final double MAX_ANGULAR_ACCELERATION_RAD = Units.degreesToRadians(862);
    public static final double NOMINAL_VOLTAGE_VOLTS = 11.5;
  }

  // Camera Constants
  public static class PhotonVisionConstants {
    //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System
    //The Cameras are mounted on the back of the value so all transform signs are flipped (not rotations). + ➔ -
    
    public static final Transform3d ROBOT_TO_CENTRAL_CAMERA =
        new Transform3d(Units.inchesToMeters(10.18917), Units.inchesToMeters(-12.61233), Units.inchesToMeters(8.62949), new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(45)));
    public static final Transform3d ROBOT_TO_LEFT_CAMERA =
        new Transform3d(Units.inchesToMeters(10.97833), Units.inchesToMeters(12.87533),  Units.inchesToMeters(8.62949), new Rotation3d(0, Units.degreesToRadians(10), 0));
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
    public static final double L1Position = -19;
    public static final double L2Position = -29;
    public static final double L3Position = -50.5; //-49.1 is perfect from oxford, our pid is bad
    public static final double L4Position = -50.5;
    //public static final double L4Position = -83;  //-81.15 is perfect from oxford, our pid is bad
    public static final double HomePosition = -2;
    
    public static final double MarginOfError = 3;

    public static final double ElevatorJoystickSpeedNerf = 0.4;
  }

  // Shooter BOXX Constants
  public static class shooterBoxxContants {
    public static final int ShooterMotorId = 39;
    public static final double kShooterMotorNominalVoltageConstant = 10.5;
    public static final int kBoxxCoralSensorChannel = 2;
    public static final int kElevatorCoralSensorChannel = 1;
    public static final double kSpitSpeed = 0.3;
    public static final double kSuckSpeed = -0.5;
    public static final double kFinalSpeed = -0.2;
  }
  public static class ClimbContants {
    public static final int ClimbMotorId = 4000;
    public static final int ClimbHomeSetpoint = 44;
    public static final int ClimbLockInSetPoint = 44;
    public static final int ClimbUpSetPoint = 44;
    public static final double ClimbP = 0.05;
    public static final double ClimbI = 0;
    public static final double ClimbD = 0;
  
  

  }
  
  //Coral Scoring Constants 
  public static class ScoreCoralConstants { 
    public static final double ElevatorError = 3;
    public static final double DriveTrainError = Units.inchesToMeters(1);
    
  }
  // Get Coral Constants
  public static class GetCoral {
  public static final double GetCoralError = Units.inchesToMeters(4);
    
  }


  public static class ControllerConstants {
    public static enum EVERYTHING_ENUM {SCORE, LEFT_SOURCE, RIGHT_SOURCE, NET, PROCESSOR, CLIMB};
  }
    
}


