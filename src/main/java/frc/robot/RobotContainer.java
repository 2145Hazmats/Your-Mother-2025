// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PoseConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController P1controller = new CommandXboxController(0);
    private final CommandXboxController P2controller = new CommandXboxController(0);
    private final CommandXboxController P3controller = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // We need to initialize an object of the camera subsystem, we don't have to use it
    private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(drivetrain);

    public RobotContainer() {
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-P1controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    //.withCenterOfRotation(Translation2d) // FOUND THIS. WILL BE USEFUL FOR DEFENCE SWERVE oR MAYBE SPINNING AROUND AN OBJECT
            )
        );

        P1controller.a().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.AngularSpeedToFaceReef()) // Drive counterclockwise with negative X (left)
            //.withCenterOfRotation(Translation2d) // FOUND THIS. WILL BE USEFUL FOR DEFENCE SWERVE oR MAYBE SPINNING AROUND AN OBJECT
    ));
        // P1controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // P1controller.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-P1controller.getLeftY(), -P1controller.getLeftX()))
        // ));
        // P1controller.x().whileTrue(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-m_CameraSubsystem.PIDDriveToPointX(15.6) * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-m_CameraSubsystem.PIDDriveToPointY(5.3) * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(m_CameraSubsystem.PIDDriveToPointROT(180) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //             //.withCenterOfRotation(Translation2d) // FOUND THIS. WILL BE USEFUL FOR DEFENCE SWERVE oR MAYBE SPINNING AROUND AN OBJICT
        //     )
        // );
        
        
       
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        P1controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        P1controller.y().whileTrue(drivetrain.followPath("TestPath")).onFalse(drivetrain.stopCommand());
        P1controller.x().whileTrue(drivetrain.pathFindThenFollowPath("TestPath")).onFalse(drivetrain.stopCommand());
        //P1controller.b().whileTrue(drivetrain.pathFindToPose(new Pose2d(9, 4, new Rotation2d(0)))).onFalse(drivetrain.stopCommand());
        
        P1controller.leftTrigger().onTrue(Commands.runOnce(() -> drivetrain.poseIndexSwitch(false)));
        P1controller.rightTrigger().onTrue(Commands.runOnce(() -> drivetrain.poseIndexSwitch(true)));

        // you can call .andThen() on autobuilder to call a command when the first one ends (bot is near the setpoint)
        P1controller.povUp().whileTrue(drivetrain.pathFindToReefCD().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(3.7) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(3) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointROT(-120) * MaxAngularRate)))).onFalse(drivetrain.stopCommand());
        P1controller.povRight().whileTrue(drivetrain.pathFindToReefCD()).onFalse(drivetrain.stopCommand());
        P1controller.povLeft().whileTrue(drivetrain.pathFindToReefEF()).onFalse(drivetrain.stopCommand());
        P1controller.povDown().whileTrue(drivetrain.pathFindToReefGH()).onFalse(drivetrain.stopCommand());
        //P1controller.povDown().whileTrue(drivetrain.pathFindToReefIJ()).onFalse(drivetrain.stopCommand());
        //P1controller.povDown().whileTrue(drivetrain.pathFindToReefKL()).onFalse(drivetrain.stopCommand());
        P1controller.b().whileTrue(drivetrain.pathFindToAllTheReefs()).onFalse(drivetrain.stopCommand());


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        //return new PathPlannerAuto("Scarlette's Road");
        return autoChooser.getSelected();
    }
}
