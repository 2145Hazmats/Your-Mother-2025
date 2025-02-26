// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.commands.ScoreCoralAuton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterBoxx;

public class NetSideTwoPieceBlue extends SequentialCommandGroup {
  private final static double fakeMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final static double fakeMaxAngularRate = RotationsPerSecond.of(DrivetrainConstants.MAX_ROTATIONS_PER_SECOND).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final static SwerveRequest.FieldCentric fakeDrive = new SwerveRequest.FieldCentric()
          .withDeadband(fakeMaxSpeed * 0.05).withRotationalDeadband(fakeMaxAngularRate * 0.05) // 10% deadband changed to 5%
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private static int preloadReefIndex = 9;
  private static int secondCoralReefIndex = 10;
  private static int levelIndex = 4;

  public NetSideTwoPieceBlue(CommandSwerveDrivetrain theLegs, ElevatorSubsystem theElephant, ShooterBoxx theSnout) {
    super(theLegs.pathFindToReefBlueIJ(),
      Commands.deadline(
        new ScoreCoralAuton(theLegs, theElephant, theSnout, preloadReefIndex, levelIndex),
        theLegs.applyRequest(() ->
          fakeDrive.withVelocityX(theLegs.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[preloadReefIndex].getX()) * fakeMaxSpeed)
              .withVelocityY(theLegs.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[preloadReefIndex].getY()) * fakeMaxSpeed)
              .withRotationalRate(theLegs.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[preloadReefIndex].getRotation().getDegrees()))
        )
      ),
      theLegs.pathFindToLeftBlueCoralStation(),
      Commands.deadline(
        theSnout.SuckTillSensor(),
        theLegs.applyRequest(() ->
          fakeDrive.withVelocityX(theLegs.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * fakeMaxSpeed)
              .withVelocityY(theLegs.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * fakeMaxSpeed)
              .withRotationalRate(theLegs.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
        )
      ),
      theLegs.pathFindToReefBlueKL(),
      Commands.deadline(
        new ScoreCoralAuton(theLegs, theElephant, theSnout, secondCoralReefIndex, levelIndex),
        theLegs.applyRequest(() ->
          fakeDrive.withVelocityX(theLegs.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[secondCoralReefIndex].getX()) * fakeMaxSpeed)
              .withVelocityY(theLegs.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[secondCoralReefIndex].getY()) * fakeMaxSpeed)
              .withRotationalRate(theLegs.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[secondCoralReefIndex].getRotation().getDegrees()))
        )
      )
    );
  }
}