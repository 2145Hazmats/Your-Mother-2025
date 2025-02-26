// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ScoreCoralConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.shooterBoxxContants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterBoxx;

// This command drives up to the reef and it moves the elevator up to the spot depending on what the reef 
// index says then it drops the coral on the branches.
public class ScoreCoralAuton extends Command {
  // Declare other subsystems 
  private CommandSwerveDrivetrain theLegs;
  private ElevatorSubsystem theElephant;
  private ShooterBoxx theSnout;

  // Declare the variables for desired Drivetrain positions and elevator height
  double xGoal;
  double yGoal;
  double degGoal;
  double elevatorGoal;

  int reefIndex;
  int levelIndex;

  // Constructor
  public ScoreCoralAuton(CommandSwerveDrivetrain theFakeLegs, ElevatorSubsystem theFakeElephant, ShooterBoxx theFakeSnout, int reefIndex, int levelIndex) {
    theLegs = theFakeLegs;
    theElephant = theFakeElephant;
    theSnout = theFakeSnout;

    this.reefIndex = reefIndex;
    this.levelIndex = levelIndex;

    addRequirements(theFakeElephant, theFakeSnout);
  }

  // Called when the command is initially scheduled.
  // sets all the variables, The command starts, The elevator moves to the desire level.
  @Override
  public void initialize() {
    // Gets desired Drivetrain positions dependant on team color 
    if (theLegs.isAllianceBlue()) {
      xGoal = PoseConstants.BLUE_REEF_POSES[reefIndex].getX();
      yGoal = PoseConstants.BLUE_REEF_POSES[reefIndex].getY();
      degGoal = PoseConstants.BLUE_REEF_POSES[reefIndex].getRotation().getDegrees();
    } else if (theLegs.isAllianceRed()) {
      xGoal = PoseConstants.RED_REEF_POSES[reefIndex].getX();
      yGoal = PoseConstants.RED_REEF_POSES[reefIndex].getY();
      degGoal = PoseConstants.RED_REEF_POSES[reefIndex].getRotation().getDegrees();
    }
    
    // Moves elevator to desired height and sets elevator goal
    if (levelIndex == 1) {theElephant.elevatorToL1(); elevatorGoal = elevatorConstants.L1Position;}
    else if (levelIndex == 2) {theElephant.elevatorToL2(); elevatorGoal = elevatorConstants.L2Position;}
    else if (levelIndex == 3) {theElephant.elevatorToL3(); elevatorGoal = elevatorConstants.L3Position;}
    else if (levelIndex == 4) {theElephant.elevatorToL4(); elevatorGoal = elevatorConstants.L4Position;}
  }

  // Every 20ms We have a PID (funny math) and we check if the height of the elevator and the postition of the robot 
  // After our conditions are satisfied we will spit out coral 
  @Override
  public void execute() {
    // Declares our Drivetrain and elevator positions
    double currentDriveX = theLegs.getPose2d().getX();
    double currentDriveY = theLegs.getPose2d().getY();
    double currentElevatorPosition = theElephant.getElevatorPosition();

    if (currentDriveX > (xGoal - ScoreCoralConstants.DriveTrainError)
    && currentDriveX < (xGoal + ScoreCoralConstants.DriveTrainError)
    && currentDriveY > (yGoal - ScoreCoralConstants.DriveTrainError)
    && currentDriveY < (yGoal + ScoreCoralConstants.DriveTrainError)) {
      theElephant.elevatorToSomething(levelIndex);
    }
    
    // Checks to see if Elevator and Drivetrain are in the correct position before playing the coral
    if (currentElevatorPosition > (elevatorGoal - ScoreCoralConstants.ElevatorError)
     && currentElevatorPosition < (elevatorGoal + ScoreCoralConstants.ElevatorError)) {
      theSnout.shootCoralMethod(); // Runs shooter if drivetrain and elevator positions are within their bounds of error
    }

    if (theSnout.BoxxCoralSensorUntriggered()) {
      theElephant.elevatorToHome();
    }
  }

  // Sends elevator to its default position after the command ends.
  @Override
  public void end(boolean interrupted) {
    theLegs.stopCommand();
    theElephant.elevatorToHome();
    theSnout.StopShooterMotor();
  }

  // Returns true when the sensor is untriggered 
  // Returns true when the command should end. Runs every 20ms
  @Override
  public boolean isFinished() {
    // THIS ENDS THE COMMAND IF THE SENSOR IS UNTRIGGERED
   if (theElephant.isDrivingSafeQuestionMark() && theSnout.BoxxCoralSensorUntriggered()) {
      return true;
    } else {
      return false;
    }
  }
}
