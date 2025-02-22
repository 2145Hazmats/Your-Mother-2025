// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
//import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimbContants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.shooterBoxxContants;
import frc.robot.Constants.ControllerConstants.EVERYTHING_ENUM;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.ReefConstants.ReefMathConstants;
import frc.robot.commands.GetCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterBoxx;

public class RobotContainer {

    private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static final double MaxAngularRate = RotationsPerSecond.of(DrivetrainConstants.MAX_ROTATIONS_PER_SECOND).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private EVERYTHING_ENUM selectedEnum;

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
   
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final SwerveRequest.RobotCentric driveCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController P1controller = new CommandXboxController(0);
    private final CommandXboxController P2controller = new CommandXboxController(1);
    private final CommandXboxController P3controller = new CommandXboxController(2);
    private final CommandXboxController P4controller = new CommandXboxController(3);
    private final CommandXboxController P5controller = new CommandXboxController(4);
    
    // We need to initialize an object of the camera subsystem, we don't have to use it
    private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(drivetrain);
    private ShooterBoxx m_ShooterBoxx = new ShooterBoxx();
    private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    //private ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

    public RobotContainer() {
        configureBindings();

        // NamedCommands.registerCommand("Elevator2Home", m_ElevatorSubsystem.elevatorToHome());
        // NamedCommands.registerCommand("Elevator2L2", m_ElevatorSubsystem.elevatorToL2());
        // NamedCommands.registerCommand("Elevator2L3", m_ElevatorSubsystem.elevatorToL3());
        // NamedCommands.registerCommand("Elevator2L4", m_ElevatorSubsystem.elevatorToL3());
        
        // NamedCommands.registerCommand("SuckTillSensor", m_ShooterBoxx.SuckTillSensor());
        // NamedCommands.registerCommand("ShootTillSensor", m_ShooterBoxx.SpitTillSensor());

        autoChooser = AutoBuilder.buildAutoChooser();
        

        SmartDashboard.putData("Auto Chooser", autoChooser);

        ReefConstants.displayReefMath();
        SmartDashboard.putBoolean("SCORE", false);
        SmartDashboard.putBoolean("LEFT_SOURCE", false);
        SmartDashboard.putBoolean("RIGHT_SOURCE", false);
        SmartDashboard.putBoolean("NET", false);
        SmartDashboard.putBoolean("PROCESSOR", false);
        SmartDashboard.putBoolean("CLIMB", false);
    }

    private void configureBindings() {

        // Default Commands :)
        //m_ElevatorSubsystem.setDefaultCommand(m_ElevatorSubsystem.defaultCommand()); TEST PID FIRST!!!!!
        //m_ClimbSubsystem.setDefaultCommand(m_ClimbSubsystem.ClimbJoystick(P2controller.getLeftY()));
        m_ShooterBoxx.setDefaultCommand(m_ShooterBoxx.StopShooterMotor());
        //m_ElevatorSubsystem.setDefaultCommand(m_ElevatorSubsystem.elevatorJoystick(P4controller::getRightY));


        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-P1controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    //.withCenterOfRotation(Translation2d)
            )
        );

        P1controller.a().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceReef() * MaxAngularRate) // Faces the Reef
        ));

        // EXAMPLE OF PID DRIVE VVVVVV

        // P1controller.x().whileTrue(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-m_CameraSubsystem.PIDDriveToPointX(15.6) * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-m_CameraSubsystem.PIDDriveToPointY(5.3) * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(m_CameraSubsystem.PIDDriveToPointROT(180) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );
        
        // EXAMPLES OF PATHFINDING/FOLLOWING PATHPLANNER
        // P1controller.y().whileTrue(drivetrain.followPath("TestPath")).onFalse(drivetrain.stopCommand());
        // P1controller.x().whileTrue(drivetrain.pathFindThenFollowPath("TestPath")).onFalse(drivetrain.stopCommand());
        // P1controller.b().whileTrue(drivetrain.pathFindToPose(new Pose2d(9, 4, new Rotation2d(0)))).onFalse(drivetrain.stopCommand());
        
        // SLOW MODE
        P1controller.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed) // Drive left with negative X (left)
            .withRotationalRate(-P1controller.getRightX() * MaxAngularRate * Constants.DrivetrainConstants.SlowMoSpeed) // Faces the Reef
        ));

        //ROBOT CENTRIC
        P1controller.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
        driveCentric.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-P1controller.getRightX() * MaxAngularRate) // Faces the Reef
        ));

        // EVERYTHING BUTTON
        // EVERYTHING_ENUM.SCORE BLUE
        P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.SCORE).and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToAllTheReefsBlue().andThen(
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getRotation().getDegrees()))
                ),
                new ScoreCoral(drivetrain, m_ElevatorSubsystem, m_ShooterBoxx)
            )
        ));
        // EVERYTHING_ENUM.SCORE RED
        P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.SCORE).and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathFindToAllTheReefsRed().andThen(
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getRotation().getDegrees()))
                ),
                new ScoreCoral(drivetrain, m_ElevatorSubsystem, m_ShooterBoxx)
            )
        ));
        // EVERYTHING_ENUM.LEFT_SOURCE
        P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.LEFT_SOURCE).and(drivetrain::isAllianceBlue)
        .whileTrue(drivetrain.pathFindToLeftBlueCoralStation().andThen(
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
                    .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
                ),
                new GetCoral(drivetrain, m_ShooterBoxx)
            )
        ));
        P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.LEFT_SOURCE).and(drivetrain::isAllianceRed)
        .whileTrue(drivetrain.pathFindToLeftRedCoralStation().andThen(
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
                    .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
                ),
                new GetCoral(drivetrain, m_ShooterBoxx)
            )
        ));
        // EVERYTHING_ENUM.RIGHT_SOURCE
        P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.RIGHT_SOURCE).and(drivetrain::isAllianceBlue)
        .whileTrue(drivetrain.pathFindToRightBlueCoralStation().andThen(
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
                    .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
                ),
                new GetCoral(drivetrain, m_ShooterBoxx)
            )
        ));
        P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.RIGHT_SOURCE).and(drivetrain::isAllianceRed)
        .whileTrue(drivetrain.pathFindToRightRedCoralStation().andThen(
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
                    .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
                ),
                new GetCoral(drivetrain, m_ShooterBoxx)
            )
        ));
        // EVERYTHING_ENUM.NET
        // P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.NET).and(drivetrain::isAllianceBlue).whileTrue(null);
        // P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.NET).and(drivetrain::isAllianceRed).whileTrue(null);
        // EVERYTHING_ENUM.PROCESSOR
        // P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.PROCESSOR).and(drivetrain::isAllianceBlue).whileTrue(null);
        // P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.PROCESSOR).and(drivetrain::isAllianceRed).whileTrue(null);
        // EVERYTHING_ENUM.CLIMB
        // P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.CLIMB).and(drivetrain::isAllianceBlue).whileTrue(null);
        // P2controller.back().and(() -> selectedEnum == EVERYTHING_ENUM.CLIMB).and(drivetrain::isAllianceRed).whileTrue(null);

        // SWITCHES POSE INDEX
        P2controller.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.poseIndexSwitch(false)));
        P2controller.rightBumper().onTrue(Commands.runOnce(() -> drivetrain.poseIndexSwitch(true)));
        // Intake and shooting coral
        P2controller.leftTrigger().whileTrue(m_ShooterBoxx.SuckTillSensor()).onFalse(m_ShooterBoxx.StopShooterMotor());
        P2controller.rightTrigger().whileTrue(m_ShooterBoxx.SpitTillSensor()).onFalse(m_ShooterBoxx.StopShooterMotor());
        // Moves Level Index 
        P2controller.povUp().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.levelIndexSwitch(true)));
        P2controller.povDown().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.levelIndexSwitch(false)));
        // Switches Enum Value (For our everything button)
        P2controller.povLeft().onTrue(Commands.runOnce(() -> { selectedEnum = EVERYTHING_ENUM.NET;        updateEnumSmartDashboard("NET");          }));
        P2controller.povRight().onTrue(Commands.runOnce(() -> { selectedEnum = EVERYTHING_ENUM.PROCESSOR; updateEnumSmartDashboard("PROCESSOR");    }));
        P2controller.a().onTrue(Commands.runOnce(() -> { selectedEnum = EVERYTHING_ENUM.SCORE;            updateEnumSmartDashboard("SCORE");        }));
        P2controller.x().onTrue(Commands.runOnce(() -> { selectedEnum = EVERYTHING_ENUM.LEFT_SOURCE;      updateEnumSmartDashboard("LEFT_SOURCE");  }));
        P2controller.y().onTrue(Commands.runOnce(() -> { selectedEnum = EVERYTHING_ENUM.CLIMB;            updateEnumSmartDashboard("CLIMB");        }));
        P2controller.b().onTrue(Commands.runOnce(() -> { selectedEnum = EVERYTHING_ENUM.RIGHT_SOURCE;     updateEnumSmartDashboard("RIGHT_SOURCE"); }));
        
        // MANUAL MODE!!!!!! :)
        P2controller.start().whileTrue(m_ElevatorSubsystem.elevatorJoystick(P2controller::getRightY));

        // RED SIDE REEF POSE METHOD
        P3controller.b().and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathFindToAllTheReefsRed().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getRotation().getDegrees()))
        )));
        // BLUE SIDE REEF POSE METHOD
        P3controller.b().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToAllTheReefsBlue().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getRotation().getDegrees()))
        )));

        // BLUE SIDE LEFT CORAL STATION
        // P3controller.leftTrigger().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToLeftBlueCoralStation().andThen(drivetrain.applyRequest(() ->
        // drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
        //     .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
        //     .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
        // )));
        
        // // BLUE SIDE RIGHT CORAL STATION
        // P3controller.rightTrigger().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToRightBlueCoralStation().andThen(drivetrain.applyRequest(() ->
        // drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
        //     .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
        //     .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
        // )));

        // RED SIDE LEFT CORAL STATION
        P3controller.leftTrigger().and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathFindToLeftRedCoralStation().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
        )));

        // RED SIDE RIGHT CORAL STATION
        P3controller.rightTrigger().and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathFindToRightRedCoralStation().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
        )));

        // Faces Left Coral Station
        P3controller.povUp().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P3controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P3controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceLeftCoralStation()) 
        ));

        // Faces Right Coral Station
        P3controller.povDown().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P3controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P3controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceRightCoralStation()) 
        ));

        // Faces Net
        P3controller.y().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P3controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P3controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceNet()) 
        ));
        //CLIMB BUTTONS  - possibly add maneeeeeeel speed control before Oxford
        //P3controller.b().onTrue(m_ClimbSubsystem.ClimbLockIn());
        //P3controller.y().onTrue(m_ClimbSubsystem.ClimbUp());

        //ELEVATOR BUTTONS
        P4controller.povDown().whileTrue(m_ElevatorSubsystem.elevatorToL1());
        
        P4controller.povLeft().whileTrue(m_ElevatorSubsystem.elevatorToL2());

        P4controller.povRight().whileTrue(m_ElevatorSubsystem.elevatorToL3());
        
        //P4controller.povUp().whileTrue(m_ElevatorSubsystem.elevatorToL4());

        P4controller.y().whileTrue(m_ElevatorSubsystem.setElevatorPID());

        P4controller.a().whileTrue(m_ElevatorSubsystem.elevatorToHome());
        // P3controller.povUp().onFalse(m_ElevatorSubsystem.elevatorToHome()); THIS WILL HIT BOTCAVE CEILING !! FIX LATER !!

        // SHOOTER BOX COMMANDS

        // P4controller.leftTrigger().whileTrue(m_ShooterBoxx.RunShooter(-0.1));
        
        
        P4controller.leftBumper().whileTrue(m_ShooterBoxx.SpitTillSensor());
        P4controller.rightBumper().whileTrue(m_ShooterBoxx.SuckTillSensor());

        //P4controller.a().onTrue(m_ElevatorSubsystem.setElevatorPID());
        P4controller.b().onTrue(m_ElevatorSubsystem.resetElevatorPosition());
        P4controller.x().whileTrue(m_ShooterBoxx.worksShoot());
        //P4controller.y().whileTrue(m_ShooterBoxx.worksRegurgitate());
        P5controller.a().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.test(), m_ElevatorSubsystem));
        
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        P5controller.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        P5controller.povDown().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        P5controller.povRight().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        P5controller.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        P5controller.y().onTrue(Commands.runOnce(() -> drivetrain.stopLogging()));
    }

    private void updateEnumSmartDashboard(String enumString) {
        SmartDashboard.putBoolean("SCORE", false);
        SmartDashboard.putBoolean("LEFT_SOURCE", false);
        SmartDashboard.putBoolean("RIGHT_SOURCE", false);
        SmartDashboard.putBoolean("NET", false);
        SmartDashboard.putBoolean("PROCESSOR", false);
        SmartDashboard.putBoolean("CLIMB", false);
        switch (enumString) {
            case "SCORE":
                SmartDashboard.putBoolean("SCORE", true);
                break;
            case "LEFT_SOURCE":
                SmartDashboard.putBoolean("LEFT_SOURCE", true);
                break;
            case "RIGHT_SOURCE":
                SmartDashboard.putBoolean("RIGHT_SOURCE", true);
                break;
            case "NET":
                SmartDashboard.putBoolean("NET", true);
                break;
            case "PROCESSOR":
                SmartDashboard.putBoolean("PROCESSOR", true);
                break;
            case "CLIMB":
                SmartDashboard.putBoolean("CLIMB", true);
                break;
            default:
                break;
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}