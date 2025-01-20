// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
    public static class PathPlannerConstants { //ONLY AFFECTS DRIVE TO POSE, NOT AUTO PATHS
      public static final double MAX_VELOCITY_MPS = 5.450;
      public static final double MAX_ACCELERATION_MPS = 3.6;
      public static final double MAX_ANGULAR_VELOCITY_RAD = Units.degreesToRadians(360);
      public static final double MAX_ANGULAR_ACCELERATION_RAD = Units.degreesToRadians(862);
      public static final double NOMINAL_VOLTAGE_VOLTS = 11.5;
    }

    public static class PhotonVisionConstants {
    //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System
    //The Cameras are mounter on the back of the value so all transform signs are flipped (not rotations). + ➔ -
    public static final Transform3d ROBOT_TO_CENTRAL_CAMERA =
        new Transform3d(Units.inchesToMeters(12), 0, -Units.inchesToMeters(4.5), new Rotation3d(0, 180, 0)); //12 inches
  } //STOLE THIS. THE NUMBERS ARE WRONNNNGGGGGG!!!!!!!!

}
