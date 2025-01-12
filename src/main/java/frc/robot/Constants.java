// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

    public static class PhotonVisionConstants {
    //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System
    //The Cameras are mounter on the back of the value so all transform signs are flipped (not rotations). + ➔ -
    public static final Transform3d ROBOT_TO_CENTRAL_CAMERA =
        new Transform3d(-0.2545401, 0.1467405, 0.1934088, new Rotation3d(0, 0, 0));
  } //STOLE THIS. THE NUMBERS ARE WRONNNNGGGGGG!!!!!!!!

}
