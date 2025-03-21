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
public class AlgaeOff extends Command {
  // Declare other subsystems 
  private CommandSwerveDrivetrain theLegs;
  private ElevatorSubsystem theElephant;
  private ShooterBoxx theSnout;
  private AlgaeSubsystem theBigStick;

  // Declare the variables for desired Drivetrain positions and elevator height
  double xGoal;
  double yGoal;
  double elevatorGoal;

  int level = 0;

  boolean areWeHigh;
  //double heightToCancel;

  // Constructor
  public AlgaeOff(CommandSwerveDrivetrain theFakeLegs, ElevatorSubsystem theFakeElephant, ShooterBoxx theFakeSnout, AlgaeSubsystem theFakeBigStick) {
    theLegs = theFakeLegs;
    theElephant = theFakeElephant;
    theSnout = theFakeSnout;
    theBigStick = theFakeBigStick;

    //addRequirements(theFakeElephant, theFakeSnout); //why drivetrain not requirement
    addRequirements(theFakeElephant);
    addRequirements(theFakeBigStick);
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
    
    
    

    // HIGH ALGAE
    if (theLegs.getPlayer1ReefIndex() == 0 || theLegs.getPlayer1ReefIndex() == 4 || theLegs.getPlayer1ReefIndex() == 8) {
      areWeHigh = true;
     
    }
    // LOW ALGAE
    if (theLegs.getPlayer1ReefIndex() == 2 || theLegs.getPlayer1ReefIndex() == 6 || theLegs.getPlayer1ReefIndex() == 10) {
      areWeHigh = false;
      
    }
  

    if (areWeHigh == true) {
      //heightToCancel = Constants.elevatorConstants.L4Position;
      level = 4;
      elevatorGoal = elevatorConstants.L4Position;
    } 
      else {
        //heightToCancel = Constants.elevatorConstants.L3Position;
        level = 3;
        elevatorGoal = elevatorConstants.L3Position;
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


    // if (theElephant.getElevatorPosition() < elevatorConstants.SAFETY_LEVEL && !theSnout.getEitherSensor()) {
    //   theElephant.elevatorToLevel(Constants.elevatorConstants.HomePosition);
    // }
    // if  (currentElevatorPosition > (elevatorGoal - ErrorConstants.ElevatorError)
    //     && currentElevatorPosition < (elevatorGoal + ErrorConstants.ElevatorError)
    //     && currentDriveX > (xGoal - ErrorConstants.DriveTrainScoreError)
    //     && currentDriveX < (xGoal + ErrorConstants.DriveTrainScoreError)
    //     && currentDriveY > (yGoal - ErrorConstants.DriveTrainScoreError)
    //     && currentDriveY < (yGoal + ErrorConstants.DriveTrainScoreError)
    //     && isArmOut ==true) {
    //   //theSnout.fireNow = true;
    //   theBigStick.MoveArmToPointMethod(Constants.AlgaeConstants.DealgifyPosition);
    //   theElephant.elevatorToLevel(0);


    // } else
      if  (currentElevatorPosition > (elevatorGoal - ErrorConstants.ElevatorError)
      && currentElevatorPosition < (elevatorGoal + ErrorConstants.ElevatorError)
      && currentDriveX > (xGoal - ErrorConstants.DriveTrainScoreError)
      && currentDriveX < (xGoal + ErrorConstants.DriveTrainScoreError)
      && currentDriveY > (yGoal - ErrorConstants.DriveTrainScoreError)
      && currentDriveY < (yGoal + ErrorConstants.DriveTrainScoreError)
      && theBigStick.isArmInDealgifyPosition()) {
        theElephant.elevatorToHomeMethod();
      }

     else if  (currentElevatorPosition > (elevatorGoal - ErrorConstants.ElevatorError)
        && currentElevatorPosition < (elevatorGoal + ErrorConstants.ElevatorError)
        && currentDriveX > (xGoal - ErrorConstants.DriveTrainScoreError)
        && currentDriveX < (xGoal + ErrorConstants.DriveTrainScoreError)
        && currentDriveY > (yGoal - ErrorConstants.DriveTrainScoreError)
        && currentDriveY < (yGoal + ErrorConstants.DriveTrainScoreError)) {
      //theSnout.fireNow = true;
      theBigStick.MoveArmToPointMethod(Constants.AlgaeConstants.DealgifyPosition);
      


    } else if (currentDriveX > (xGoal - ErrorConstants.DriveTrainElevatorUpError)
        && currentDriveX < (xGoal + ErrorConstants.DriveTrainElevatorUpError)
        && currentDriveY > (yGoal - ErrorConstants.DriveTrainElevatorUpError)
        && currentDriveY < (yGoal + ErrorConstants.DriveTrainElevatorUpError)
        && theElephant.isElevatorHome()) {
      theElephant.elevatorToLevel(level);
    } 
        }

  // Sends elevator to its default position after the command ends.
  @Override
  public void end(boolean interrupted) {
    theElephant.elevatorToLevel(Constants.elevatorConstants.HomePosition);
    theBigStick.MoveArmToPointMethod(AlgaeConstants.HomePosition);
  }

  // Returns true when the sensor is untriggered and the elevator is up
  // Returns true when the command should end. Runs every 20ms
  @Override
  public boolean isFinished() {
    //return (false); 
    return (theElephant.isElevatorHome() && theBigStick.IsArmAwayFromHome()); 
  }
}
