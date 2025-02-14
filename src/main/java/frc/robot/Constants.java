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
    public static final int motorLeaderID = 99;
    public static final int motorFollowerID = 98;

    // PID
    public static final double PIDBottom = 0;
    public static final double PIDL2 = 0;
    public static final double PIDL3 = 0;
    public static final double PIDL4 = 0;

    // Elevator Level Heights
    public static final double L1Position = 0;
    public static final double L2Position = 0;
    public static final double L3Position = 0;
    public static final double L4Position = 0;
    public static final double HomePosition = 0;
  }

  // Shooter BOXX Constants
  public static class shooterBoxxContants {
    public static final int ShooterMotorId = 400;
    
  } 
}
