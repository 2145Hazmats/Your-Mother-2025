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

    // PID Constants
    public static final double PID_XY = 0.55;
    public static final double PID_DEGREE = 0.008;
    public static final double PID_RAD = 6;

    // Feed Forward Constants
    public static final double FEEDFORWARD_CONSTANT = 0.022;
    public static final double FEEDFORWARD_CONSTANT_DEGREE = 0.12;
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
        new Transform3d(Units.inchesToMeters(-12.1), 0, -Units.inchesToMeters(7.55), new Rotation3d(0, Math.PI, 0));
    public static final Transform3d ROBOT_TO_LEFT_CAMERA =
    new Transform3d(-0.2545401, -0.1467405,  0.1934088, new Rotation3d(0, Units.degreesToRadians(210), Units.degreesToRadians(-12)));
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
    public static final double ElaphantP = 0.5;
    public static final double ElaphantI = 0;
    public static final double ElaphantD = 0;

    // Elevator Level Heights
    public static final double L1Position = -15;
    public static final double L2Position = -34;
    public static final double L3Position = -48;
    public static final double L4Position = -64;
    public static final double HomePosition = -5;
    
    public static final double MarginOfError = 3;
  }

  // Shooter BOXX Constants
  public static class shooterBoxxContants {
    public static final int ShooterMotorId = 39;
    public static final double kShooterMotorNominalVoltageConstant = 10.5;
    public static final int kCoralSensorChannel = 0;
    public static final double kSpitSpeed = 0.5;
    public static final double kSuckSpeed = -0.5;
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
    public static final double ElevatorError = Units.inchesToMeters(3);
    public static final double DriveTrainError = Units.inchesToMeters(3);
    
  }

  public static class ControllerConstants {
    public static enum EVERYTHING_ENUM {SCORE, LEFT_SOURCE, RIGHT_SOURCE, NET, PROCESSOR, CLIMB};
  }
    
}


