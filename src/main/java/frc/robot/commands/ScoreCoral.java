// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterBoxx;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  private CommandSwerveDrivetrain theLegs;
  private ElevatorSubsystem theElephant;
  private ShooterBoxx theSnout;

  private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  
  
  public double heightGoal;
  double xgoal;
  double ygoal;
  double x;
  double y;
    

  /** Creates a new ScoreCoral. */
  public ScoreCoral(CommandSwerveDrivetrain theFakeLegs, ElevatorSubsystem theFakeElephant, ShooterBoxx theFakeSnout) {
    theLegs = theFakeLegs;
    theElephant = theFakeElephant;
    theSnout = theFakeSnout;

   
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(theFakeLegs, theFakeElephant, theFakeSnout);
    
    // This is what you would do to get the level
    //double desiredLevel = theElaphant.getLevel();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xgoal = PoseConstants.RED_REEF_POSES[theLegs.getReefIndex()].getX();
    ygoal = PoseConstants.RED_REEF_POSES[theLegs.getReefIndex()].getY();
    x = theLegs.getPose2d().getX();
    y = theLegs.getPose2d().getY();

    heightGoal = theElephant.getHeightGoal();
    theElephant.elevatorToL1();
    
    RobotContainer.drive
    .withVelocityX(theLegs.PIDDriveToPointX(xgoal) * MaxSpeed)
    .withVelocityY(theLegs.PIDDriveToPointY(ygoal) * MaxSpeed)
    .withRotationalRate(theLegs.angularSpeedToFaceReef());

    
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (theElephant.getElevatorPosition() > heightGoal 
    && x > xgoal- Constants.DrivetrainConstants.MarginOfError
    && x < xgoal + Constants.DrivetrainConstants.MarginOfError);
     
    

    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
