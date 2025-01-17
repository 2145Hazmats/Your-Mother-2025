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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {
  private CommandSwerveDrivetrain charizardsSkateboard = null;

  private final Field2d visionField2d = new Field2d();
  private final Field2d generalField = new Field2d();
  private PhotonCamera centralCamera = new PhotonCamera("Middle_Arducam_OV9281");
  private PhotonPipelineResult centralResult = null;
  private PhotonTrackedTarget centralTarget = null;
  private final PIDController pidControllerX = new PIDController(1.65, 0, 0);//.3
  private final PIDController pidControllerY = new PIDController(1.65, 0, 0);//.3
  private final PIDController pidControllerRot = new PIDController(0.01, 0, 0);

  // PhotonVision objects used in vision localization
  private PhotonPoseEstimator centralPoseEstimator = new PhotonPoseEstimator(
    AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), //TODO: 2025
    //Calculates a new robot position estimate by combining all visible tag corners.
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    Constants.PhotonVisionConstants.ROBOT_TO_CENTRAL_CAMERA);

  private EstimatedRobotPose centralLatestRobotPose = null;

  private Matrix<N3, N1> curStdDevs;
  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);//4,4,8
  private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);//0.5,0.5,1

  private Field2d soloVisionField = new Field2d();

  /* Constructor */
  public CameraSubsystem(CommandSwerveDrivetrain drivetrain) {
    charizardsSkateboard = drivetrain;
  }

  /* Add a vision measurement for localization */ //TODO: Modify
  public void addVisionPose2d(Pose2d pose2d, double timestampSeconds) {
    SmartDashboard.putNumber("aVP2d pose2d X", pose2d.getX());
    SmartDashboard.putNumber("aVP2d pose2d Y", pose2d.getY());
    SmartDashboard.putNumber("aVP2d pose2d Rot", pose2d.getRotation().getDegrees());
    SmartDashboard.putNumber("aVP2d timestampSeconds", timestampSeconds);
    // Sets trust value for vision measurements
    charizardsSkateboard.setVisionMeasurementStdDevs(curStdDevs);
    charizardsSkateboard.addVisionMeasurement(pose2d, timestampSeconds);
  }
  
  public double getPoseX() {
    return charizardsSkateboard.getState().Pose.getX();
 };
 public double getPoseY() {
  return charizardsSkateboard.getState().Pose.getY();
};
public double getPoseRot() {
  return charizardsSkateboard.getState().Pose.getRotation().getDegrees();
};

/*public double PIDDriveToPoint(double DesiredPoseX, double DesiredPoseY, double DesiredPoseAngle) {

  double SpeedsForPose = pidController.calculate(getPoseX(), DesiredPoseX);
}*/ //HOW DO I DO THIS WITH ONE METHOD?
public double PIDDriveToPointX(double DesiredPoseX) {
  double SpeedsForPose = pidControllerX.calculate(getPoseX(), DesiredPoseX);

  SpeedsForPose = SpeedsForPose + Math.signum(SpeedsForPose) * .016;
  return SpeedsForPose; 

}
  public double PIDDriveToPointY(double DesiredPoseY) {
    double SpeedsForPose = pidControllerY.calculate(getPoseY(), DesiredPoseY);
    SpeedsForPose = SpeedsForPose + Math.signum(SpeedsForPose)* .016;
    return SpeedsForPose; 
}
public double PIDDriveToPointROT(double DesiredPoseRot) {
  double SpeedsForPose = pidControllerRot.calculate(Math.abs(getPoseRot()), DesiredPoseRot); //only works with 180
  SpeedsForPose = SpeedsForPose * Math.signum(getPoseRot());
  SpeedsForPose = SpeedsForPose + Math.signum(SpeedsForPose)* .016;
  return SpeedsForPose;
  
}
  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
          Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
          // No pose input. Default to single-tag std devs
          curStdDevs = kSingleTagStdDevs;

      } else {
          // Pose present. Start running Heuristic
          var estStdDevs = kSingleTagStdDevs;
          int numTags = 0;
          double avgDist = 0;

          // Precalculation - see how many tags we found, and calculate an average-distance metric
          for (var tgt : targets) {
              var tagPose = centralPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
              if (tagPose.isEmpty()) continue;
              numTags++;
              avgDist +=
                      tagPose
                              .get()
                              .toPose2d()
                              .getTranslation()
                              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          }

          if (numTags == 0) {
              // No tags visible. Default to single-tag std devs
              curStdDevs = kSingleTagStdDevs;
          } else {
              // One or more tags visible, run the full heuristic.
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

  @Override
  public void periodic() {
    centralResult = centralCamera.getLatestResult();
    SmartDashboard.putBoolean("CameraTrue", centralCamera.getLatestResult().hasTargets());
    
    //Angle off tag for lining up with one tag
    // if (centralCamera.getLatestResult().hasTargets()) {
    //   SmartDashboard.putNumber("CCYaw", centralCamera.getLatestResult().getBestTarget().getYaw());
    // };
    //Field Updates
    
    SmartDashboard.putData("VisionField", soloVisionField);
    SmartDashboard.putData("GeneralField", generalField);

    // Try to update "latestRobotPose" with a new "EstimatedRobotPose" using a "PhotonPoseEstimator"
    // If "latestRobotPose" is updated, call addVisionPose2d() and pass the updated "latestRobotPose" as an argument
    try {
      centralLatestRobotPose = centralPoseEstimator.update(centralResult).get();
      updateEstimationStdDevs(centralPoseEstimator.update(centralResult), centralCamera.getAllUnreadResults().get(0).getTargets());
      addVisionPose2d(centralLatestRobotPose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds()); //centralLatestRobotPose.timestampSeconds);
      soloVisionField.setRobotPose(centralLatestRobotPose.estimatedPose.toPose2d());
      SmartDashboard.putNumber("Vision X", centralLatestRobotPose.estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Vision Y", centralLatestRobotPose.estimatedPose.toPose2d().getY());
      SmartDashboard.putNumber("Vision Rot", centralLatestRobotPose.estimatedPose.toPose2d().getRotation().getDegrees());
      SmartDashboard.putBoolean("middleLatestRobotPose Update", true);
    } catch (Exception e) {
      centralLatestRobotPose = null;
      SmartDashboard.putBoolean("middleLatestRobotPose Update", false);
    }

    SmartDashboard.putNumber("charizardsSkateboard X", charizardsSkateboard.getState().Pose.getX());
    SmartDashboard.putNumber("charizardsSkateboard Y", charizardsSkateboard.getState().Pose.getY());
    SmartDashboard.putNumber("charizardsSkateboard Rot", charizardsSkateboard.getState().Pose.getRotation().getDegrees());

    generalField.setRobotPose(charizardsSkateboard.getState().Pose);

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Match Number", DriverStation.getMatchNumber());
  }
}
