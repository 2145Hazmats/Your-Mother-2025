// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
//import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbContants;
import frc.robot.Constants.shooterBoxxContants;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.ReefConstants.ReefMathConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterBoxx;

public class RobotContainer {

    private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
     
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
   
    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final SwerveRequest.RobotCentric driveCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController P1controller = new CommandXboxController(0);
    private final CommandXboxController P2controller = new CommandXboxController(1);
    
    // We need to initialize an object of the camera subsystem, we don't have to use it
    private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(drivetrain);
    private ShooterBoxx m_ShooterBoxx = new ShooterBoxx();
    private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();


    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        ReefConstants.displayReefMath();
    }

    private void configureBindings() {

        // Default Commands :)
        //m_ElevatorSubsystem.setDefaultCommand(m_ElevatorSubsystem.defaultCommand()); TEST PID FIRST!!!!!
        m_ClimbSubsystem.setDefaultCommand(m_ClimbSubsystem.ClimbJoystick(P2controller.getLeftY()));
        m_ShooterBoxx.setDefaultCommand(m_ShooterBoxx.StopShooterMotor());
        m_ElevatorSubsystem.setDefaultCommand(m_ElevatorSubsystem.elevatorJoystick(P2controller.getRightY()));


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
            .withRotationalRate(drivetrain.angularSpeedToFaceReef()) // Faces the Reef
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

        // SWITCHES POSE INDEX
        P1controller.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.poseIndexSwitch(false)));
        P1controller.rightBumper().onTrue(Commands.runOnce(() -> drivetrain.poseIndexSwitch(true)));
    
        // RED SIDE REEF POSE METHOD
        P1controller.b().and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathFindToAllTheReefsRed().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[drivetrain.getReefIndex()].getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.angularSpeedToFaceReef())
        )));

        // BLUE SIDE REEF POSE METHOD
        P1controller.x().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToAllTheReefsBlue().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[drivetrain.getReefIndex()].getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.angularSpeedToFaceReef())
        )));

        // BLUE SIDE LEFT CORAL STATION
        // P1controller.leftTrigger().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToLeftBlueCoralStation().andThen(drivetrain.applyRequest(() ->
        // drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
        //     .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
        //     .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
        // )));
        
        // // BLUE SIDE RIGHT CORAL STATION
        // P1controller.rightTrigger().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToRightBlueCoralStation().andThen(drivetrain.applyRequest(() ->
        // drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
        //     .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
        //     .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
        // ))); STOLE THE BUTTONS LOL GOOD LUCK

        // RED SIDE LEFT CORAL STATION
        P1controller.leftTrigger().and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathFindToLeftRedCoralStation().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
        )));

        // RED SIDE RIGHT CORAL STATION
        P1controller.rightTrigger().and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathFindToRightRedCoralStation().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
        )));

        // Faces Left Coral Station
        P1controller.povUp().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceLeftCoralStation()) 
        ));

        // Faces Right Coral Station
        P1controller.povDown().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceRightCoralStation()) 
        ));

        // Faces Net
        P1controller.y().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceNet()) 
        ));
        //CLIMB BUTTONS  - possibly add maneeeeeeel speed control before Oxford
        P2controller.b().onTrue(m_ClimbSubsystem.ClimbLockIn());
        P2controller.y().onTrue(m_ClimbSubsystem.ClimbUp());

        //ELEVATOR BUTTONS
        P2controller.povDown().onTrue(m_ElevatorSubsystem.elevatorToL1());
        P2controller.povDown().onFalse(m_ElevatorSubsystem.elevatorToHome());
        
        P2controller.povLeft().onTrue(m_ElevatorSubsystem.elevatorToL2());
        P2controller.povLeft().onFalse(m_ElevatorSubsystem.elevatorToHome());

        P2controller.povRight().onTrue(m_ElevatorSubsystem.elevatorToL3());
        P2controller.povRight().onFalse(m_ElevatorSubsystem.elevatorToHome());

        // P2controller.povUp().onTrue(m_ElevatorSubsystem.elevatorToL4());
        // P2controller.povUp().onFalse(m_ElevatorSubsystem.elevatorToHome()); THIS WILL HIT BOTCAVE CEILING !! FIX LATER !!

        // SHOOTER BOX COMMANDS

        P2controller.leftTrigger().onTrue(m_ShooterBoxx.RunShooter(-0.3));
        P2controller.leftTrigger().onFalse(m_ShooterBoxx.StopShooterMotor());

        P2controller.rightTrigger().onTrue(m_ShooterBoxx.RunShooter(0.3)).onFalse(m_ShooterBoxx.StopShooterMotor());
        
        P2controller.leftBumper().whileTrue(m_ShooterBoxx.SuckTillSensor());
        P2controller.rightBumper().whileTrue(m_ShooterBoxx.SuckTillSensor());
        
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
       
     }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}