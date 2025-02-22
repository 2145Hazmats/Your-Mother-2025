// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;

public class CameraSubsystem extends SubsystemBase {
  private CommandSwerveDrivetrain charizardsSkateboard = null;

  private final Field2d visionField = new Field2d();
  private final Field2d generalField = new Field2d();
  
  private boolean imNotReady;
  // Central Camera
  private final PhotonCamera centralCamera = new PhotonCamera("Middle_Arducam_OV9281");
  private PhotonPipelineResult centralResult = null;
  private PhotonTrackedTarget centralTarget = null;
  private PhotonPoseEstimator centralPoseEstimator = new PhotonPoseEstimator(
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    PhotonVisionConstants.ROBOT_TO_CENTRAL_CAMERA
  );
  private EstimatedRobotPose centralEstimatedRobotPose = null;

  // Left Camera
  private final PhotonCamera leftCamera = new PhotonCamera("Left_Arducam_OV9281");
  private PhotonPipelineResult leftResult = null;
  private PhotonTrackedTarget leftTarget = null;
  private PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    PhotonVisionConstants.ROBOT_TO_LEFT_CAMERA
  );
  private EstimatedRobotPose leftEstimatedRobotPose = null;
  
  private Matrix<N3, N1> curStdDevs;
  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  // TODO : edit starting values
  private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  /* Constructor */
  public CameraSubsystem(CommandSwerveDrivetrain drivetrain) {
    charizardsSkateboard = drivetrain;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) { curStdDevs = kSingleTagStdDevs; }
    else {
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      for (var tgt : targets) { // Precalculation - see how many tags we found, and calculate an average-distance metric
        var tagPose = centralPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) { // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else { // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
              estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDevs = estStdDevs;
      }
    }
  }
  
  /* Add a vision measurement for localization */
  public void addVisionPose2d(Pose2d pose2d, double timestampSeconds) {
    SmartDashboard.putNumber("aVP2d pose2d X", pose2d.getX());
    SmartDashboard.putNumber("aVP2d pose2d Y", pose2d.getY());
    SmartDashboard.putNumber("aVP2d pose2d Rot", pose2d.getRotation().getDegrees());
    SmartDashboard.putNumber("aVP2d timestampSeconds", timestampSeconds);
    // Sets trust value for vision measurements
    charizardsSkateboard.setVisionMeasurementStdDevs(curStdDevs);
    charizardsSkateboard.addVisionMeasurement(pose2d, timestampSeconds);
  }
  
  // Returns Pose Values
  public Pose2d getPose2d() { return charizardsSkateboard.getState().Pose; }
  public double getPoseX() { return charizardsSkateboard.getState().Pose.getX(); }
  public double getPoseY() { return charizardsSkateboard.getState().Pose.getY(); }
  public double getPoseRot() { return charizardsSkateboard.getState().Pose.getRotation().getDegrees(); }


  public Command imNotReady(boolean ready) {
    return Commands.runOnce(() -> imNotReady = ready, this);
  }

  public boolean checkReady() {
    if (imNotReady = true) {
      return false;
    } else {
      return true;
    }

  }
  @Override
  public void periodic() {
    // Get camera results
    centralResult = centralCamera.getLatestResult();
    leftResult = leftCamera.getLatestResult();

    // Central Camera
    // Try to update "latestRobotPose" with a new "EstimatedRobotPose" using a "PhotonPoseEstimator"
    // If "latestRobotPose" is updated, call addVisionPose2d() and pass the updated "latestRobotPose" as an argument
    try {
      centralEstimatedRobotPose = centralPoseEstimator.update(centralResult).get();
      updateEstimationStdDevs(centralPoseEstimator.update(centralResult), centralCamera.getAllUnreadResults().get(0).getTargets());
      addVisionPose2d(centralEstimatedRobotPose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
      visionField.setRobotPose(centralEstimatedRobotPose.estimatedPose.toPose2d());
      
      SmartDashboard.putNumber("Central Vision X", centralEstimatedRobotPose.estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Central Vision Y", centralEstimatedRobotPose.estimatedPose.toPose2d().getY());
      SmartDashboard.putNumber("Central Vision Rot", centralEstimatedRobotPose.estimatedPose.toPose2d().getRotation().getDegrees());
      SmartDashboard.putBoolean("middleLatestRobotPose Update", true);
    } catch (Exception e) {
      centralEstimatedRobotPose = null;
      SmartDashboard.putBoolean("middleLatestRobotPose Update", false);
    }

    // Same thing but for the left camera
    try {
      leftEstimatedRobotPose = leftPoseEstimator.update(leftResult).get();
      updateEstimationStdDevs(leftPoseEstimator.update(leftResult), leftCamera.getAllUnreadResults().get(0).getTargets());
      addVisionPose2d(leftEstimatedRobotPose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
      visionField.setRobotPose(leftEstimatedRobotPose.estimatedPose.toPose2d());

      SmartDashboard.putNumber("Left Vision X", leftEstimatedRobotPose.estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Left Vision Y", leftEstimatedRobotPose.estimatedPose.toPose2d().getY());
      SmartDashboard.putNumber("Left Vision Rot", leftEstimatedRobotPose.estimatedPose.toPose2d().getRotation().getDegrees());
      SmartDashboard.putBoolean("leftLatestRobotPose Update", true);
    } catch (Exception e) {
      leftEstimatedRobotPose = null;
      SmartDashboard.putBoolean("leftLatestRobotPose Update", false);
    }

    //Updates smartdashboard
    SmartDashboard.putBoolean("CameraMidTrue", centralCamera.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("CameraLeftTrue", leftCamera.getLatestResult().hasTargets());

    SmartDashboard.putNumber("charizardsSkateboard X", charizardsSkateboard.getState().Pose.getX());
    SmartDashboard.putNumber("charizardsSkateboard Y", charizardsSkateboard.getState().Pose.getY());
    SmartDashboard.putNumber("charizardsSkateboard Rot", charizardsSkateboard.getState().Pose.getRotation().getDegrees());

    SmartDashboard.putData("VisionField", visionField);
    SmartDashboard.putData("GeneralField", generalField);
    generalField.setRobotPose(charizardsSkateboard.getState().Pose);
  }
}