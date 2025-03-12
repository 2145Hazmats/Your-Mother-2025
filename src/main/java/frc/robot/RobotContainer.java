// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.shooterBoxxContants;
import frc.robot.Constants.ControllerConstants.EVERYTHING_ENUM;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.commands.FireCoralAuton;
import frc.robot.commands.ScoreAlgaeNet;
//import frc.robot.autos.AwesomestAutoBlue;
// import frc.robot.autos.AwesomeAuton;
// import frc.robot.autos.NetSideAuto;
// import frc.robot.autos.ProcessorSideAuto;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoralAuton;
import frc.robot.commands.ScoreCoralManual;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystemNeo;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.ShooterBoxx;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    public final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(DrivetrainConstants.MAX_ROTATIONS_PER_SECOND).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private EVERYTHING_ENUM selectedEnum;

    //private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
   
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final SwerveRequest.RobotCentric driveCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake drivePointWheelsAt = new SwerveRequest.SwerveDriveBrake();

   // private SendableChooser<Command> pathPlannerAutoChooser;
    //private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController P1controller = new CommandXboxController(0);
    private final CommandXboxController P2controller = new CommandXboxController(1);
    private final CommandXboxController P3controller = new CommandXboxController(2);
    //private final CommandXboxController P4controller = new CommandXboxController(3);
    //private final CommandPS4Controller p5Controller = new CommandPS4Controller(4);
    //private final Trigger disableCameras = overrides.cameraSwitch();
    
    // We need to initialize an object of the camera subsystem, we don't have to use it
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(m_drivetrain);
    private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private ShooterBoxx m_ShooterBoxx = new ShooterBoxx(m_ElevatorSubsystem);
    private ClimbSubsystemNeo m_ClimbSubsystemNeo = new ClimbSubsystemNeo();
    //private AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
    //private ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

    private Indexing m_indexing = new Indexing(m_ElevatorSubsystem, m_drivetrain);

    public RobotContainer() {
        configureBindings();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

          NamedCommands.registerCommand("Elevator2Home", m_ElevatorSubsystem.elevatorToHome());
          NamedCommands.registerCommand("Elevator2L2", m_ElevatorSubsystem.elevatorToL2().withTimeout(3));
          NamedCommands.registerCommand("Elevator2L3", m_ElevatorSubsystem.elevatorToL3());
          NamedCommands.registerCommand("Elevator2L4", m_ElevatorSubsystem.elevatorToL3());
        
          //NamedCommands.registerCommand("SuckTillCoralSensor", m_ShooterBoxx.SuckTillCoralSensorAuto());
          //NamedCommands.registerCommand("SuckTillElevatorSensor", m_ShooterBoxx.SuckTillElevatorSensorAuto());

          NamedCommands.registerCommand("SuckTillSensor", m_ShooterBoxx.SuckTillCoralSensorAuto()); //OG Command Depricating soon
          NamedCommands.registerCommand("ShootTillSensor", m_ShooterBoxx.SpitTillSensor());
          NamedCommands.registerCommand("SpitOnFloor", Commands.run(() -> m_ShooterBoxx.RunShooter(shooterBoxxContants.kSpitSpeed), m_ShooterBoxx).withTimeout(2));


          NamedCommands.registerCommand("AutoL4", new ScoreCoralAuton( m_ElevatorSubsystem, m_ShooterBoxx, 4)); //.withTimeout(2).finallyDo(() -> m_ElevatorSubsystem.elevatorToHome()));
          NamedCommands.registerCommand("FireL4", new FireCoralAuton( m_ElevatorSubsystem, m_ShooterBoxx, 4  ));
          
        // NamedCommands.registerCommand("ReadyToLeaveStation", m_ShooterBoxx.ElevatorCoralSensorTriggered() );

         autoChooser = AutoBuilder.buildAutoChooser();
         SmartDashboard.putData("Auto Chooser", autoChooser);
        // SmartDashboard.putData("pathPlannerAutoChooser", autoChooser);

        // autoChooser.setDefaultOption("AwesomeAuton", new AwesomeAuton(m_drivetrain,m_ElevatorSubsystem,m_ShooterBoxx));
        // autoChooser.addOption("NetSideAuto", new NetSideAuto(m_drivetrain,m_ElevatorSubsystem,m_ShooterBoxx));
        // autoChooser.addOption("ProcessorSideAuto", new ProcessorSideAuto());
        //autoChooser.addOption("Net Side Auto", new netSideAuto());
        //autoChooser.addOption("Processor Side Auto", processorSideAuto());
        //SmartDashboard.putData("autoChooser", autoChooser);
    }

    public CommandSwerveDrivetrain getSwerveDrivetrain() {
        return m_drivetrain;
    }

    private void configureBindings() {
        // Default Commands :)
        m_ElevatorSubsystem.setDefaultCommand(Commands.either(m_ElevatorSubsystem.defaultCommand(), m_ElevatorSubsystem.elevatorJoystick(P3controller::getLeftY), m_indexing::isP2ManualModeFalse)); //NEEDS TESTING
        m_ShooterBoxx.setDefaultCommand(Commands.either(m_ShooterBoxx.IntakeSolosDefaultCommand(), Commands.run(() -> m_ShooterBoxx.stopShooterMethod(), m_ShooterBoxx), m_indexing::isP2ManualModeFalse));
        m_ClimbSubsystemNeo.setDefaultCommand(m_ClimbSubsystemNeo.Keepclimbsafe());

        m_drivetrain.registerTelemetry(logger::telemeterize);

        m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1controller.getLeftX() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-P1controller.getRightX() * MaxAngularRate * m_ElevatorSubsystem.getElevatorSlowSpeed()) // Drive counterclockwise with negative X (left)
            )
        );

        //-------------------------------P1 Controls---------------------------------


        // Manual Controls
        P2controller.povDown().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(1)),
         m_ElevatorSubsystem.elevatorToL1(), m_indexing::isP1ManualModeFalse));
        // LOCK THE WHEELS
        P1controller.povLeft().whileTrue(m_drivetrain.applyRequest(() -> drivePointWheelsAt));

        // LEFT SOURCE BLUE
        P1controller.leftBumper().and(m_drivetrain::isAllianceBlue).whileTrue(
            m_drivetrain.pathFindToLeftBlueCoralStation().andThen(
            Commands.parallel(
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
                 )
                
            )
        ));

        // LEFT SOURCE RED
        P1controller.leftBumper().and(m_drivetrain::isAllianceRed).whileTrue(
            m_drivetrain.pathFindToLeftRedCoralStation().andThen(
            Commands.parallel(
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
                 )
                
            )
        ));
        
        // RIGHT SOURCE BLUE
        P1controller.rightBumper().and(m_drivetrain::isAllianceBlue).whileTrue(
            m_drivetrain.pathFindToRightBlueCoralStation().andThen(
            Commands.parallel(
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
                 )
                 
            )
        ));

        // RIGHT SOURCE RED
        P1controller.rightBumper().and(m_drivetrain::isAllianceRed).whileTrue(
            m_drivetrain.pathFindToRightRedCoralStation().andThen(
             Commands.parallel(
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
                 )
                 
            )
        ));
        
        // REEF SCORING

        // SCORE BLUE
        P1controller.leftTrigger().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToAllTheReefsBlue().andThen(//.onlyIf(() -> m_ShooterBoxx.getEitherSensor()).andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                )
            )
        ));

        // REEF SCORE RED
        P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToAllTheReefsRed().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                )
            )
        ));

        // // NET SCORE BLUE
        // P1controller.a().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToBlueNet().andThen(
        //     Commands.deadline(
        //         new ScoreAlgaeNet(m_drivetrain, m_ElevatorSubsystem, m_AlgaeSubsystem),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.NET_BLUE_POSE.getX()) * MaxSpeed)
        //             // .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.NET_BLUE_POSE.getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.NET_BLUE_POSE.getRotation().getDegrees()))
        //         )
        //     )
        // ));

        // // NET SCORE RED
        // P1controller.a().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToRedNet().andThen(
        //     Commands.deadline(
        //         new ScoreAlgaeNet(m_drivetrain, m_ElevatorSubsystem, m_AlgaeSubsystem),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.NET_RED_POSE.getX()) * MaxSpeed)
        //             // .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.NET_RED_POSE.getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.NET_RED_POSE.getRotation().getDegrees()))
        //         )
        //     )
        // ));

        // SLOW MODE
        P1controller.rightTrigger().whileTrue(m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
                .withVelocityY(-P1controller.getLeftX() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
                .withRotationalRate(-P1controller.getRightX() * MaxAngularRate * Constants.DrivetrainConstants.SlowMoSpeed)
        ));

        // FACE LEFT CORAL STATION
        P1controller.x().whileTrue(m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withVelocityY(-P1controller.getLeftX() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withRotationalRate(-m_drivetrain.angularSpeedToFaceLeftCoralStation() * m_ElevatorSubsystem.getElevatorSlowSpeed())
        ));

        // FACE RIGHT CORAL STATION
        P1controller.b().whileTrue(m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withVelocityY(-P1controller.getLeftX() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withRotationalRate(-m_drivetrain.angularSpeedToFaceRightCoralStation() * m_ElevatorSubsystem.getElevatorSlowSpeed())
        ));

        // FACE REEF
        // P1controller.a().whileTrue(m_drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
        //         .withVelocityY(-P1controller.getLeftX() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
        //         .withRotationalRate(-m_drivetrain.angularSpeedToFaceReef() * m_ElevatorSubsystem.getElevatorSlowSpeed())
        // ));

        // FACE NET
        P1controller.y().whileTrue(m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withVelocityY(-P1controller.getLeftX() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withRotationalRate(-m_drivetrain.angularSpeedToFaceNet() * m_ElevatorSubsystem.getElevatorSlowSpeed())
        ));

        // CENTRIC MODE
        // P1controller.leftTrigger().whileTrue(m_drivetrain.applyRequest(() ->
        //  driveCentric.withVelocityX(-P1controller.getLeftY() * Constants.DrivetrainConstants.SlowMoSpeed) // Drive forward with negative Y (forward)
        //      .withVelocityY(-P1controller.getLeftX() * Constants.DrivetrainConstants.SlowMoSpeed) // Drive left with negative X (left)
        //      .withRotationalRate(-P1controller.getRightX() * Constants.DrivetrainConstants.SlowMoSpeed) // Faces the Reef
        //  ));

        //-------------------------------P2 Controls---------------------------------
        
        // Elevator
        
        //P2controller.povDown().whileTrue(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(1)));
        P2controller.povLeft().whileTrue(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(2)));
        P2controller.povRight().whileTrue(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(3)));
        P2controller.povUp().whileTrue(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(4)));

         P2controller.povDown().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(1)),
         m_ElevatorSubsystem.elevatorToL1(), m_indexing::isP2ManualModeFalse));
/* 
         P2controller.povLeft().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(2)),
         m_ElevatorSubsystem.elevatorToL2(), m_indexing::isManualModeFalse));

         P2controller.povRight().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(3)),
         m_ElevatorSubsystem.elevatorToL3(), m_indexing::isManualModeFalse));

         P2controller.povUp().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(4)),
         m_ElevatorSubsystem.elevatorToL4(), m_indexing::isManualModeFalse));
*/

                
        // Reef Index

        P2controller.leftBumper().onTrue(Commands.runOnce(() -> m_indexing.poseIndexSwitch(false)));
        P2controller.rightBumper().onTrue(Commands.runOnce(() -> m_indexing.poseIndexSwitch(true)));

        // Climb using Neo

        P2controller.leftTrigger().whileTrue(Commands.run(() ->m_ClimbSubsystemNeo.climbInCommand(), m_ClimbSubsystemNeo));
        //P2controller.rightTrigger().whileTrue(Commands.run(() -> m_ClimbSubsystemNeo.PutTheServoInTheRightSpotPlease(), m_ClimbSubsystemNeo).until(()->m_ClimbSubsystemNeo.ReadyToStickTheClimbOutIGuess()).andThen(Commands.waitSeconds(4)).andThen(()-> m_ClimbSubsystemNeo.climbForwardCommand()));//.andThen(() ->m_ClimbSubsystemNeo.climbForwardCommand()));
        P2controller.x().whileTrue(Commands.run(() ->m_ClimbSubsystemNeo.PutTheServoInTheRightSpotPlease(), m_ClimbSubsystemNeo));
        P2controller.rightTrigger().whileTrue(Commands.run(() ->m_ClimbSubsystemNeo.climbOutCommand(), m_ClimbSubsystemNeo));
        // Climb using Servo
        //P2controller.b().whileTrue(Commands.run(() -> m_ClimbSubsystemNeo.resetMotorPosition(), m_ClimbSubsystemNeo));
        //P2controller.back().whileFalse(m_ClimbSubsystemNeo.climbLock());

        //P2controller.back().whileTrue(m_ClimbSubsystemNeo.climbUnlock());


        // Send values to P1

        P2controller.a().whileTrue(Commands.runOnce(() -> m_indexing.updateP1Index()));

        // Stop Elevator

        P2controller.start().whileTrue(m_ElevatorSubsystem.disableElevator());
        P2controller.back().toggleOnTrue(Commands.startEnd(() -> m_indexing.setP2ManualModeYes(), () -> m_indexing.setP2ManualModeNo(), m_indexing));
        

        P2controller.y().whileTrue(m_ElevatorSubsystem.elevatorToHome());

        //P2controller.b().whileTrue(m_ShooterBoxx.RunShooter(shooterBoxxContants.kSuckSpeed)).onFalse(m_ShooterBoxx.RunShooter(0));
        //P2controller.b().whileTrue(m_ShooterBoxx.IntakeDefaultCommand()).onFalse(m_ShooterBoxx.StopShooterMotor());

        // P2controller.b().whileTrue(
        //         m_ShooterBoxx.BanditSetIntakeMotorCommand(Constants.shooterBoxxContants.kSuckSpeed).until(m_ShooterBoxx::BanditNoteSensorTriggered)
        //       );
        P2controller.b().whileTrue(m_ShooterBoxx.worksShoot());
              //.until(m_ShooterBoxx.BanditNoteSensorTriggered());//BanditNoteSensorTriggered);//m_ShooterBoxx::BanditNoteSensorTriggered

        //P2controller.back().whileTrue(m_ShooterBoxx.SpitTillSensor());

        // 

        //-------------------------------PS4 Controls EXPERIMENTAL--------------------
        
        //p5Controller.setRumble(RumbleType.kBothRumble, 1);

        // -----------------------------Manual Stuff---------------------------------
        
        // Intake and shooting coral
        //P3controller.y().whileTrue(m_ShooterBoxx.RunShooter(-.4));
        P3controller.y().whileTrue(Commands.run(() ->m_ClimbSubsystemNeo.climbToSetpointPID(), m_ClimbSubsystemNeo));
        P3controller.b().whileTrue(Commands.run(() -> m_ClimbSubsystemNeo.climbToNailItPID(), m_ClimbSubsystemNeo));
        //P3controller.b().whileTrue(m_ShooterBoxx.RunShooter(-.6));
        //P3controller.a().whileTrue(m_ShooterBoxx.SuckTillSensor());
        P3controller.x().whileTrue(m_ShooterBoxx.SpitTillSensor());

        
        // Elevator
        P3controller.back().whileTrue(m_ElevatorSubsystem.elevatorJoystick(P3controller::getLeftY)); //Not sure if this will work needs testing
        P3controller.povDown().whileTrue(m_ElevatorSubsystem.elevatorToL1());
        P3controller.povLeft().whileTrue(m_ElevatorSubsystem.elevatorToL2());
        P3controller.povRight().whileTrue(m_ElevatorSubsystem.elevatorToL3());
        P3controller.povUp().whileTrue(m_ElevatorSubsystem.elevatorToL4());

        P3controller.a().whileTrue(new ScoreCoralAuton( m_ElevatorSubsystem, m_ShooterBoxx,  4).withTimeout(1.75));
        P3controller.start().whileTrue(Commands.run(() -> m_ShooterBoxx.SuckTillCoralSensorDerekSkillIssueFix(), m_ShooterBoxx));
        //CLIMB
        
        //P3controller.rightTrigger
        //().whileTrue(m_ClimbSubsystem.ClimbUp());
        //P3controller.leftTrigger().whileTrue(m_ClimbSubsystem.ClimbToHome());
        //P3controller.start().whileTrue(m_ClimbSubsystem.ClimbLockIn());

        //P3controller.leftTrigger().whileTrue(m_ShooterBoxx.SuckTillSensor()).onFalse(m_ShooterBoxx.StopShooterMotor());
        //P3controller.rightTrigger().whileTrue(m_ShooterBoxx.SpitTillSensor()).onFalse(m_ShooterBoxx.StopShooterMotor());
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
        //return new AwesomestAutoBlue(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx);
        
    }
}