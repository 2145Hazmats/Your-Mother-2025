// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ErrorConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterBoxx;

// This command drives up to the reef and it moves the elevator up to the spot depending on what the reef 
// index says then it drops the coral on the branches.
public class ScoreCoral extends Command {
  // Declare other subsystems 
  private CommandSwerveDrivetrain theLegs;
  private ElevatorSubsystem theElephant;
  private ShooterBoxx theSnout;
  private AlgaeSubsystem theAlgae;

  // Declare the variables for desired Drivetrain positions and elevator height
  double xGoal;
  double yGoal;
  double elevatorGoal;

  int level = 0;

  boolean algaeMode = false;
  boolean areWeHigh;

  // Constructor
  public ScoreCoral(CommandSwerveDrivetrain theFakeLegs, ElevatorSubsystem theFakeElephant, ShooterBoxx theFakeSnout, AlgaeSubsystem theFakeAlgae) {
    theLegs = theFakeLegs;
    theElephant = theFakeElephant;
    theSnout = theFakeSnout;
    theAlgae = theFakeAlgae;

    //addRequirements(theFakeElephant, theFakeSnout); //why drivetrain not requirement
    addRequirements(theFakeElephant, theFakeAlgae);
  }

  // Called when the command is initially scheduled.
  // sets all the variables, The command starts, The elevator moves to the desire level.
  @Override
  public void initialize() {
    // Gets desired Drivetrain positions dependant on team color 
    if (theLegs.isAllianceBlue()) {
      xGoal = PoseConstants.BLUE_REEF_POSES[theLegs.getPlayer1ReefIndex()].getX();
      yGoal = PoseConstants.BLUE_REEF_POSES[theLegs.getPlayer1ReefIndex()].getY();
    } else if (theLegs.isAllianceRed()) {
      xGoal = PoseConstants.RED_REEF_POSES[theLegs.getPlayer1ReefIndex()].getX();
      yGoal = PoseConstants.RED_REEF_POSES[theLegs.getPlayer1ReefIndex()].getY();
    }
    
    // Retrives our desired level index
    level = theElephant.getPlayer1LevelIndex();
    
    // Moves elevator to desired height and sets elevator goal
    if (level == 1) {elevatorGoal = elevatorConstants.L1Position;}
    else if (level == 2) {elevatorGoal = elevatorConstants.L2Position;}
    else if (level == 3) {elevatorGoal = elevatorConstants.L3Position;}
    else if (level == 4) {elevatorGoal = elevatorConstants.L4Position;}

    algaeMode = false;

    if ((theLegs.getPlayer1ReefIndex() % 2) == 0 && level == 4) {
      algaeMode = true;
    }

    // HIGH ALGAE
    if (theLegs.getPlayer1ReefIndex() == 0 || theLegs.getPlayer1ReefIndex() == 4 || theLegs.getPlayer1ReefIndex() == 8) {
      areWeHigh = true;
    }
    
    // LOW ALGAE
    if (theLegs.getPlayer1ReefIndex() == 2 || theLegs.getPlayer1ReefIndex() == 6 || theLegs.getPlayer1ReefIndex() == 10) {
      areWeHigh = false;
    }
  }

  // Every 20ms We have a PID (funny math) and we check if the height of the elevator and the postition of the robot 
  // After our conditions are satisfied we will spit out coral
  @Override
  public void execute() {
    // Declares our Drivetrain and elevator positions
    double currentDriveX = theLegs.getPose2d().getX();
    double currentDriveY = theLegs.getPose2d().getY();
    double currentElevatorPosition = theElephant.getElevatorPosition();

    boolean isElevatorSensorTrue = theSnout.getElevatorSensor();
    //boolean isCoralSensorTrue = theSnout.getBoxxSensor();

    SmartDashboard.putBoolean("elevator value", isElevatorSensorTrue);
    //SmartDashboard.putBoolean("coral value", isCoralSensorTrue);

    SmartDashboard.putNumber("ScoreCoral X", currentDriveX - xGoal);
    SmartDashboard.putNumber("ScoreCoral Y", currentDriveY - yGoal);
    SmartDashboard.putNumber("ScoreCoral Elevator", currentElevatorPosition - elevatorGoal);

    // if ((theLegs.getPlayer1ReefIndex() % 2) == 0 && level == 4) { // && magicBoolean == true) {
    //   // HIGH ALGAE
    //   if (theLegs.getPlayer1ReefIndex() == 0 || theLegs.getPlayer1ReefIndex() == 4 || theLegs.getPlayer1ReefIndex() == 8) {

    //   }
    //   // LOW ALGAE
    //   if (theLegs.getPlayer1ReefIndex() == 2 || theLegs.getPlayer1ReefIndex() == 6 || theLegs.getPlayer1ReefIndex() == 10) {
        
    //   }
    // }

    if (theElephant.getElevatorPosition() < elevatorConstants.SAFETY_LEVEL && !theSnout.getEitherSensor()) {
      theElephant.elevatorToLevel(Constants.elevatorConstants.HomePosition);
    }

    if (currentElevatorPosition > (elevatorGoal - ErrorConstants.ElevatorError)
        && currentElevatorPosition < (elevatorGoal + ErrorConstants.ElevatorError)
        && currentDriveX > (xGoal - ErrorConstants.DriveTrainScoreError)
        && currentDriveX < (xGoal + ErrorConstants.DriveTrainScoreError)
        && currentDriveY > (yGoal - ErrorConstants.DriveTrainScoreError)
        && currentDriveY < (yGoal + ErrorConstants.DriveTrainScoreError)) {
      theSnout.fireNow = true;
      if (algaeMode == true) {
        theAlgae.MoveArmToPointMethod(AlgaeConstants.DealgifyPosition);
      }
    } else if (currentDriveX > (xGoal - ErrorConstants.DriveTrainElevatorUpError)
        && currentDriveX < (xGoal + ErrorConstants.DriveTrainElevatorUpError)
        && currentDriveY > (yGoal - ErrorConstants.DriveTrainElevatorUpError)
        && currentDriveY < (yGoal + ErrorConstants.DriveTrainElevatorUpError)
        && !isElevatorSensorTrue
        && theElephant.isElevatorHome()) {
      theElephant.elevatorToLevel(level);
    }
  }

  // Sends elevator to its default position after the command ends.
  @Override
  public void end(boolean interrupted) {
    theElephant.elevatorToLevel(Constants.elevatorConstants.HomePosition);
    theAlgae.MoveArmToPointMethod(AlgaeConstants.HomePosition);
  }

  // Returns true when the sensor is untriggered and the elevator is up
  // Returns true when the command should end. Runs every 20ms
  @Override
  public boolean isFinished() {
    if (algaeMode && areWeHigh) {
      return (theElephant.isElevatorInDealgifyPositionHigh() && !theSnout.getEitherSensor()); //(theElephant.isElevatorHome() && theAlgae.IsArmAwayFromHome() && !theSnout.getEitherSensor());
    } else if (algaeMode && (areWeHigh == false)) {
      return (theElephant.isElevatorInDealgifyPositionLow() && !theSnout.getEitherSensor());
    } else {
      return (theElephant.getElevatorPosition() > elevatorConstants.SAFETY_LEVEL && !theSnout.getEitherSensor());
    }
  }
}
