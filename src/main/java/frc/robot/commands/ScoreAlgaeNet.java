// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ErrorConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAlgaeNet extends Command {
  private CommandSwerveDrivetrain drive;
  private ElevatorSubsystem elevator;
  private AlgaeSubsystem algae;

  double xGoal;
  double degGoal;
  double elevatorGoal;
  
  /** Creates a new ScoreAlgae. */
  public ScoreAlgaeNet(CommandSwerveDrivetrain drivetemp, ElevatorSubsystem elevatortemp, AlgaeSubsystem algaetemp) {
    drive = drivetemp;
    elevator = elevatortemp;
    algae = algaetemp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, algae);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (drive.isAllianceBlue()) {
      xGoal = AlgaeConstants.BLUE_NET_POSES.getX();
    } else if (drive.isAllianceRed()) {
      xGoal = AlgaeConstants.RED_NET_POSES.getX();
    }
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDriveX = drive.getPose2d().getX();
    double currentElevatorPosition = elevator.getElevatorPosition();
    double currentAlgaeArmPosition = algae.getAlgaeArmPosition();

    if (currentDriveX > (xGoal - ErrorConstants.DriveTrainScoreError)
        && currentDriveX < (xGoal + ErrorConstants.DriveTrainScoreError))
    {
      elevator.elevatorToSomething(elevatorConstants.L4Position);
      algae.moveArmToPointMethod(AlgaeConstants.NetPosition);
    }
        
    if (currentElevatorPosition > (elevatorGoal - ErrorConstants.ElevatorError)
        && currentElevatorPosition < (elevatorGoal + ErrorConstants.ElevatorError)
        && currentAlgaeArmPosition > (currentAlgaeArmPosition - AlgaeConstants.ALGAE_ARM_ERROR)
        && currentAlgaeArmPosition < (currentAlgaeArmPosition + AlgaeConstants.ALGAE_ARM_ERROR)
        && currentDriveX > (xGoal - ErrorConstants.DriveTrainScoreError)
        && currentDriveX < (xGoal + ErrorConstants.DriveTrainScoreError))
    {
      algae.outtakeAlgaeMethod();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorToSomething(elevatorConstants.HomePosition);
    algae.moveArmToPointMethod(AlgaeConstants.HomePosition);
    algae.stopAlgaeShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
