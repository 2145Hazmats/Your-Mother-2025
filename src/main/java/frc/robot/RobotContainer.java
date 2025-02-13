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
    private final CommandXboxController P2controller = new CommandXboxController(1);
    private final CommandXboxController P3controller = new CommandXboxController(2);

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
        drivetrain.registerTelemetry(logger::telemeterize);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
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
            .withRotationalRate(drivetrain.angularSpeedToFaceReef()) // Drive counterclockwise with negative X (left)
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
        // P1controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // P1controller.y().whileTrue(drivetrain.followPath("TestPath")).onFalse(drivetrain.stopCommand());
        // P1controller.x().whileTrue(drivetrain.pathFindThenFollowPath("TestPath")).onFalse(drivetrain.stopCommand());
        //P1controller.b().whileTrue(drivetrain.pathFindToPose(new Pose2d(9, 4, new Rotation2d(0)))).onFalse(drivetrain.stopCommand());
        
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
        P1controller.leftTrigger().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToLeftBlueCoralStation().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
        )));
        
        // BLUE SIDE RIGHT CORAL STATION
        P1controller.rightTrigger().and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathFindToRightBlueCoralStation().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
            .withVelocityY(drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
            .withRotationalRate(drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
        )));

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

        P1controller.povUp().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceLeftCoralStation()) 
        ));

        P1controller.povDown().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceRightCoralStation()) 
        ));

        P1controller.y().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.angularSpeedToFaceNet()) 
        ));

        // Intakes note into robot
        // P2controller.leftBumper().whileTrue(m_BoxSubsystem.setIntakeMotorCommandThenStop(BoxConstants.kIntakeSpeed));

        // P2controller.rightBumper().whileTrue(m_BoxSubsystem.setIntakeMotorCommandThenStop(BoxConstants.kRegurgitateSpeed));

        /* Operator Controls */

        // Sets the speed of the shooter motors and starts intake/feed motor
        // When the button is released, the arm goes to idle position and the m_box default command is ran
    //     P3controller.leftTrigger().whileTrue(
    //     m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, true)
    //     );//.onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, true));

    //     // Intakes note into robot
    //     P3controller.leftBumper().whileTrue(m_BoxSubsystem.setIntakeMotorCommand(BoxConstants.kIntakeSpeed));

    //     // Regurgitate everything
    //     P3controller.rightBumper().whileTrue(m_BoxSubsystem.YeetCommand(BoxConstants.kRegurgitateSpeed, BoxConstants.kRegurgitateSpeed));

    //     // Smartshoot button, only shoots the note when Velocity is correct and the button is held down.
    //     P3controller.rightTrigger().whileTrue(
    //     Commands.sequence(
    //         Commands.waitUntil(m_BoxSubsystem::isVelocityReached),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, true)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
    
    //     // Arm set point for climbing
    //     P3controller.button(9).whileTrue(
    //     m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.CLIMB_1, false)
    //     );

    //     //m_operatorController.button(10).onTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.CLIMB_2, true));
        
    //     // Arm set point for shooting speaker from subwoofer
    //     P3controller.a().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.SHOOT_SUB, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Arm set point for playing amp
    //     P3controller.x().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.AMP, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Idle mode arm set point
    //     P3controller.b().whileTrue(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Arm set point for shooting podium
    //     P3controller.povRight().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.SHOOT_N2, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Arm set point for shooting horizontal across the field
    //     // Commenting it out to use this button
    //     /*
    //     m_operatorController.povLeft().whileTrue(
    //     Commands.parallel(
    //         m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_HORIZONTAL, true),
    //         m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
    //     */

    //     // Arm set point for a pass shot
    //     P3controller.povLeft().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.PASS, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));


    //     // Manual control toggle for arm
    //     P3controller.start().toggleOnTrue(
    //         m_ArmSubsystem.manualArmCommand(() -> P3controller.getRightY() * Constants.ArmConstants.kManualSpeed, 
    //         () -> P3controller.getLeftY() * Constants.ArmConstants.kManualSpeed)
    //     );

    //     // Smart floor intake with regurgitate?
    //     P3controller.povDown().whileTrue(
    //     Commands.sequence(
    //         Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.FLOOR, false),
    //         m_BoxSubsystem.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
    //         ).until(m_BoxSubsystem::noteSensorTriggered)
    //     )
    //     );

    //     // Intake from the source
    //     P3controller.povUp().whileTrue(
    //     Commands.sequence(
    //         Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.SOURCE, false),
    //         m_BoxSubsystem.setIntakeMotorCommand(BoxConstants.kSourceIntakeSpeed)
    //         ).until(m_BoxSubsystem::noteSensorTriggered)
    //     )
    //     );
        
    //     // I commented the trap out to use it's keybind
    //     /*
    //     m_operatorController.povRight().whileTrue(
    //     Commands.parallel(
    //         m_arm.setArmPIDCommand(ArmConstants.ArmState.TRAP, true),
    //         m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //         )
    //     ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
    //     */

    //     // Reset wrist encoder
    //     P3controller.back().onTrue(Commands.runOnce(() -> m_ArmSubsystem.resetWristEncoder()));
     }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        //return new PathPlannerAuto("Scarlette's Road");
        return autoChooser.getSelected();
    }
}