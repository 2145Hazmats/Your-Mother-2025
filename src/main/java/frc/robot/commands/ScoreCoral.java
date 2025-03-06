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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
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
  
  // Sets max speed for drivetrain
  private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MaxAngularRate = RotationsPerSecond.of(DrivetrainConstants.MAX_ROTATIONS_PER_SECOND).in(RadiansPerSecond);
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // Declare the variables for desired Drivetrain positions and elevator height
  double xGoal;
  double yGoal;
  double degGoal;
  double elevatorGoal;

  int level = 0;

  // Constructor
  public ScoreCoral(CommandSwerveDrivetrain theFakeLegs, ElevatorSubsystem theFakeElephant, ShooterBoxx theFakeSnout) {
    theLegs = theFakeLegs;
    theElephant = theFakeElephant;
    theSnout = theFakeSnout;

    addRequirements(theFakeElephant, theFakeSnout); //why drivetrain not requirement
  }

  // Called when the command is initially scheduled.
  // sets all the variables, The command starts, The elevator moves to the desire level.
  @Override
  public void initialize() {
    // Gets desired Drivetrain positions dependant on team color 
    if (theLegs.isAllianceBlue()) {
      xGoal = PoseConstants.BLUE_REEF_POSES[theLegs.getPlayer1ReefIndex()].getX();
      yGoal = PoseConstants.BLUE_REEF_POSES[theLegs.getPlayer1ReefIndex()].getY();
      degGoal = PoseConstants.BLUE_REEF_POSES[theLegs.getPlayer1ReefIndex()].getRotation().getDegrees();
    } else if (theLegs.isAllianceRed()) {
      xGoal = PoseConstants.RED_REEF_POSES[theLegs.getPlayer1ReefIndex()].getX();
      yGoal = PoseConstants.RED_REEF_POSES[theLegs.getPlayer1ReefIndex()].getY();
      degGoal = PoseConstants.RED_REEF_POSES[theLegs.getPlayer1ReefIndex()].getRotation().getDegrees();
    }
    
    // Retrives our desired level index
    level = theElephant.getPlayer1LevelIndex();
    
    // Moves elevator to desired height and sets elevator goal
    if (level == 1) {theElephant.elevatorToL1(); elevatorGoal = elevatorConstants.L1Position;}//Try removing this
    else if (level == 2) {theElephant.elevatorToL2(); elevatorGoal = elevatorConstants.L2Position;}
    else if (level == 3) {theElephant.elevatorToL3(); elevatorGoal = elevatorConstants.L3Position;}
    else if (level == 4) {theElephant.elevatorToL4(); elevatorGoal = elevatorConstants.L4Position;}
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
      theElephant.elevatorToSomething(level);
    }
    
    // // Checks to see if Elevator and Drivetrain are in the correct position before playing the coral
    // if (currentElevatorPosition > (elevatorGoal - ScoreCoralConstants.ElevatorError)
    //  && currentElevatorPosition < (elevatorGoal + ScoreCoralConstants.ElevatorError)) {
    //   theSnout.shootCoralMethod(); // Runs shooter if drivetrain and elevator positions are within their bounds of error
    // }

    if (theSnout.BoxxCoralSensorUntriggered()) {
      theElephant.elevatorToSomething(Constants.elevatorConstants.HomePosition);//-1 Changed post comp not tested
    } else if (currentElevatorPosition > (elevatorGoal - ScoreCoralConstants.ElevatorError)
     && currentElevatorPosition < (elevatorGoal + ScoreCoralConstants.ElevatorError)) {
      theSnout.shootCoralMethod(); // Runs shooter if drivetrain and elevator positions are within their bounds of error
    }

    if (theSnout.BoxxCoralSensorUntriggered()) { //try removing not working i think
      theElephant.elevatorToHome();
    }

    SmartDashboard.putNumber("ScoreCoral level", level); //can remove this for cimp

    SmartDashboard.putBoolean("currentDriveX 1", currentDriveX > (xGoal - ScoreCoralConstants.DriveTrainError));
    SmartDashboard.putBoolean("currentDriveX 2", currentDriveX < (xGoal + ScoreCoralConstants.DriveTrainError));
    SmartDashboard.putBoolean("currentDriveY 1", currentDriveY > (yGoal - ScoreCoralConstants.DriveTrainError));
    SmartDashboard.putBoolean("currentDriveY 2", currentDriveY < (yGoal + ScoreCoralConstants.DriveTrainError));

    SmartDashboard.putBoolean("currentElevatorPosition 1", currentElevatorPosition > (elevatorGoal - ScoreCoralConstants.ElevatorError));
    SmartDashboard.putBoolean("currentElevatorPosition 2", currentElevatorPosition < (elevatorGoal + ScoreCoralConstants.ElevatorError));
  }

  // Sends elevator to its default position after the command ends.
  @Override
  public void end(boolean interrupted) {
    // theLegs.stopCommand();
    // theElephant.elevatorToHome();
    // theSnout.StopShooterMotor();
    theElephant.elevatorToHome(); //do commands not work?
    theSnout.stopShooterMethod();
  }

  // Returns true when the sensor is untriggered 
  // Returns true when the command should end. Runs every 20ms
  @Override
  public boolean isFinished() {

    
   if (theElephant.isDrivingSafeQuestionMark() && theSnout.BoxxCoralSensorUntriggered()) { //mess with driving safe method
    theSnout.stopShooterMethod();
    return true;
  } else {
    return false;
  }
}
  //   // THIS ENDS THE COMMAND IF THE SENSOR IS UNTRIGGERED
  //  if (theElephant.isDrivingSafeQuestionMark() && theSnout.BoxxCoralSensorUntriggered()) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  //}
}
