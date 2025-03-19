// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.shooterBoxxContants;
import frc.robot.Constants.ControllerConstants.EVERYTHING_ENUM;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.commands.FireCoralAuton;
import frc.robot.commands.PutElevatorUp;
//import frc.robot.autos.AwesomestAutoBlue;
// import frc.robot.autos.AwesomeAuton;
// import frc.robot.autos.NetSideAuto;
// import frc.robot.autos.ProcessorSideAuto;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoralAuton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSuperSystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CameraSubsystem;
//import frc.robot.subsystems.ChirpMusic;
import frc.robot.subsystems.ClimbSubsystemNeo;
import frc.robot.subsystems.CommandSwerveDrivetrain; //
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.ShooterBoxx;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> stationChooser;

    public final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(DrivetrainConstants.MAX_ROTATIONS_PER_SECOND).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    //private EVERYTHING_ENUM selectedEnum;
   
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final SwerveRequest.RobotCentric driveCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake lockWheels = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);


    //----new Controllers------- 1 and 2 + 4 and 5 output same joystick direction.getrawaxis
    //for bonus buttons do Trigger testbutton = nwe trigger( -> controller.getrawbutton(6))
    //private final XBOXCONTROLLER p1controller = new XBOXCONTROLER(1)
    //testbutton.whiletrue
    private final CommandXboxController P1controller = new CommandXboxController(0);
    private final CommandXboxController P2controller = new CommandXboxController(1);
    private final CommandXboxController P3controller = new CommandXboxController(2);
    private final CommandXboxController P4controller = new CommandXboxController(3);
    private final CommandXboxController P5controller = new CommandXboxController(4);
    // We need to initialize an object of the camera subsystem, we don't have to use it
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(m_drivetrain);
    
    private ClimbSubsystemNeo m_ClimbSubsystemNeo = new ClimbSubsystemNeo();
    private AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
    private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(m_AlgaeSubsystem);
    private ShooterBoxx m_ShooterBoxx = new ShooterBoxx(m_ElevatorSubsystem);
    private AlgaeSuperSystem m_AlgaeSuperSystem = new AlgaeSuperSystem(m_ElevatorSubsystem, m_AlgaeSubsystem, m_ShooterBoxx);
    private Indexing m_indexing = new Indexing(m_ElevatorSubsystem, m_drivetrain);
    // private CANdleSubsystem m_CANdle = new CANdleSubsystem(2);

    public RobotContainer() {
        configureBindings();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

          NamedCommands.registerCommand("Elevator2Home", m_ElevatorSubsystem.elevatorToHome().withTimeout(.15));
          NamedCommands.registerCommand("Elevator2L2", m_ElevatorSubsystem.elevatorToL2().withTimeout(.15));
          NamedCommands.registerCommand("Elevator2L3", m_ElevatorSubsystem.elevatorToL3().withTimeout(.15));
          NamedCommands.registerCommand("Elevator2L4", m_ElevatorSubsystem.elevatorToL3().withTimeout(.15));
        
          //NamedCommands.registerCommand("SuckTillCoralSensor", m_ShooterBoxx.SuckTillCoralSensorAuto());
          NamedCommands.registerCommand("SuckTillElevatorSensor", m_ShooterBoxx.SuckTillElevatorSensorAuto());

          NamedCommands.registerCommand("SuckTillSensor", m_ShooterBoxx.SuckTillCoralSensorAutoCommand()); //OG Command Depricating soon
          NamedCommands.registerCommand("ShootTillSensor", m_ShooterBoxx.SpitTillSensorCommand());
          NamedCommands.registerCommand("SpitOnFloor", Commands.run(() -> m_ShooterBoxx.RunShooter(shooterBoxxContants.kSpitSpeed), m_ShooterBoxx).withTimeout(2));


          //NamedCommands.registerCommand("AutoL4", new ScoreCoralAuton( m_ElevatorSubsystem, m_ShooterBoxx, 4)); //.withTimeout(2).finallyDo(() -> m_ElevatorSubsystem.elevatorToHome()));
          NamedCommands.registerCommand("AutoL4", new PutElevatorUp(m_ElevatorSubsystem, m_ShooterBoxx, 4));
          NamedCommands.registerCommand("FireL4", new FireCoralAuton(m_ElevatorSubsystem, m_ShooterBoxx, 4));
          
          //NamedCommands.registerCommand("ReadyToLeaveStation", m_ShooterBoxx.ElevatorCoralSensorTriggered());

         autoChooser = AutoBuilder.buildAutoChooser();
         SmartDashboard.putData("Auto Chooser", autoChooser);


         
         stationChooser = new SendableChooser<>();
         stationChooser.addOption("LeftStation", true);
         stationChooser.addOption("RightStation", false);
         SmartDashboard.putData("Station Chooser", stationChooser);

    }

    public CommandSwerveDrivetrain getSwerveDrivetrain() { // for coasting out of auto
        return m_drivetrain;
    }

    private void configureBindings() {
        // Default Commands :)
        m_ElevatorSubsystem.setDefaultCommand(Commands.either(m_ElevatorSubsystem.defaultCommand(),Commands.run(()-> m_ElevatorSubsystem.elevatorJoystick(P2controller.getLeftY()), m_ElevatorSubsystem) , m_indexing::isP2ManualModeFalse)); //NEEDS TESTING
        //m_ElevatorSubsystem.setDefaultCommand(Commands.either(m_ElevatorSubsystem.elevatorToL1(),m_ElevatorSubsystem.defaultCommand() , m_indexing::isP2ManualModeFalse));
        m_ShooterBoxx.setDefaultCommand(Commands.either(m_ShooterBoxx.IntakeSolosDefaultCommand(), Commands.run(() -> m_ShooterBoxx.StopShooterMethod(), m_ShooterBoxx), m_indexing::isP2ManualModeFalse));
        //m_ClimbSubsystemNeo.setDefaultCommand(m_ClimbSubsystemNeo.KeepClimbSafeDefaultCommand());
        m_AlgaeSubsystem.setDefaultCommand(Commands.run(()-> m_AlgaeSubsystem.algaeJoystick(P4controller.getRightY(),P4controller.getLeftY()), m_AlgaeSubsystem));//(MathUtil.applyDeadband(P4controller.getRightY(), 0.1)), MathUtil.applyDeadband(P4controller.getLeftY(), 0.1)));
        
    //     // Indexing LOL!!
    // m_indexing.setDefaultCommand(m_indexing.SettingReefIndexBasedOnController(P2controller::getRightX, P2controller::getRightY));

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
        // P2controller.povDown().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(1)),
        //  m_ElevatorSubsystem.elevatorToL1(), m_indexing::isP1ManualModeFalse));
        // // LOCK THE WHEELS
        // P1controller.povLeft().whileTrue(m_drivetrain.applyRequest(() -> lockWheels));

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
        // P1controller.leftTrigger().and(m_drivetrain::isAllianceBlue).and(() ->stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
        //     //m_indexing.updateP1IndexInRepeatingCommand(),
        //     m_drivetrain.pathFindToAllTheReefsBlue().andThen(
        //     Commands.deadline(
        //         new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //         ))
                
                
        //         .andThen( m_drivetrain.pathFindToLeftBlueCoralStation().andThen(
                    
        //                 m_drivetrain.applyRequest(() ->
        //                     drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
        //                     .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
        //                     .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
        //                  ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
        //         )))
        //     )
        // );


        // P1controller.leftTrigger().and(m_drivetrain::isAllianceBlue).and(() ->!stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
        //     //m_indexing.updateP1IndexInRepeatingCommand(),
        //     m_drivetrain.pathFindToAllTheReefsBlue().andThen(
        //     Commands.deadline(
        //         new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //         ))
                
                
        //         .andThen( m_drivetrain.pathFindToRightBlueCoralStation().andThen(
                    
        //                 m_drivetrain.applyRequest(() ->
        //                     drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
        //                     .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
        //                     .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
        //                  ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
        //         )))
        //     )
        // );

        // P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).and(() ->stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
        //     //m_indexing.updateP1IndexInRepeatingCommand(),
        //     m_drivetrain.pathFindToAllTheReefsRed().andThen(
        //     Commands.deadline(
        //         new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //         ))
                
                
        //         .andThen( m_drivetrain.pathFindToLeftRedCoralStation().andThen(
                    
        //                 m_drivetrain.applyRequest(() ->
        //                     drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
        //                     .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
        //                     .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
        //                  ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
        //         )))
        //     )
        // );


        // P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).and(() ->!stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
        //     //m_indexing.updateP1IndexInRepeatingCommand(),
        //     m_drivetrain.pathFindToAllTheReefsBlue().andThen(
        //     Commands.deadline(
        //         new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //         ))
                
                
        //         .andThen( m_drivetrain.pathFindToRightRedCoralStation().andThen(
                    
        //                 m_drivetrain.applyRequest(() ->
        //                     drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
        //                     .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
        //                     .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
        //                  ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
        //         )))
        //     )
        // );

        
// REEF SCORE RED
        // P1controller.a().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToAllTheReefsRed().andThen(
            
                
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                
        //     )
        // ));

        // P1controller.a().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToAllTheReefsBlue().andThen(
           
                
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                
        //     )
        // ));
        // // REEF SCORE RED
        // P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToAllTheReefsRed().andThen(
        //     Commands.deadline(
        //         new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //         )
        //     )
        // ));

        // P1controller.leftTrigger().whileTrue(Commands.startEnd(() -> m_indexing.pathFindingInProgress = true, () -> {m_indexing.pathFindingInProgress = false; m_indexing.updateP1Index();}));

        //  P3controller.leftTrigger().whileTrue(Commands.repeatingSequence(
            
        //     Commands.either((m_drivetrain.pathFindToAllTheReefsBlue().andThen(//.onlyIf(() -> m_ShooterBoxx.getEitherSensor()).andThen(
        //     Commands.deadline(
        //         new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //         )
        //     )
        // )), m_drivetrain.pathFindToAllTheReefsRed().andThen(
        //     Commands.deadline(
        //         new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //         )
        //     )
        // ), () -> m_drivetrain.isAllianceBlue()))

        // // .andThen(Commands.either(Commands.either(m_drivetrain.pathFindToLeftBlueCoralStation(),  m_drivetrain.pathFindToRightBlueCoralStation(), () -> stationChooser.getSelected()),
        // //  Commands.either( m_drivetrain.pathFindToLeftRedCoralStation(),  m_drivetrain.pathFindToRightRedCoralStation(), () -> stationChooser.getSelected()), 
        //  m_drivetrain::isAllianceBlue)
        //  )
         
        //  .andThen(Commands.either(Commands.either(Commands.parallel(
        //     m_drivetrain.applyRequest(() ->
        //         drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
        //         .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
        //         .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
        //      )
            
        // ),  Commands.parallel(
        //     m_drivetrain.applyRequest(() ->
        //         drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
        //         .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
        //         .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
        //      )
             
        // ), () -> stationChooser.getSelected()),
        //  Commands.either(  Commands.parallel(
        //     m_drivetrain.applyRequest(() ->
        //         drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
        //         .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
        //         .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
        //      )
            
        // ),  Commands.parallel(
        //     m_drivetrain.applyRequest(() ->
        //         drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
        //         .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
        //         .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
        //      )
             
        // ), () -> stationChooser.getSelected()), 
        //  m_drivetrain::isAllianceBlue)
        //  )
         
         
         
         
         
         
         //);
        // // NET SCORE BLUE
        // P1controller.a().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToBlueNet().andThen(
        //     Commands.deadline(
        //         //new ScoreAlgaeNet(m_drivetrain, m_ElevatorSubsystem, m_AlgaeSubsystem), score algea net command here
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
        //         //new ScoreAlgaeNet(m_drivetrain, m_ElevatorSubsystem, m_AlgaeSubsystem), score algae net command here
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.NET_RED_POSE.getX()) * MaxSpeed)
        //             // .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.NET_RED_POSE.getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.NET_RED_POSE.getRotation().getDegrees()))
        //         )
        //     )
        // ));

        // // PROCCESSOR SCORE BLUE
        // P1controller.a().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToBlueProcessor().andThen(
        //     Commands.deadline(
        //         //new ScoreAlgaeNet(m_drivetrain, m_ElevatorSubsystem, m_AlgaeSubsystem), score processor command here
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.PROCESSOR_BLUE_POSE.getX()) * MaxSpeed)
        //              .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.PROCESSOR_BLUE_POSE.getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.PROCESSOR_BLUE_POSE.getRotation().getDegrees()))
        //         )
        //     )
        // ));

        // // PROCESSOR SCORE RED
        // P1controller.a().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToRedProcessor().andThen(
        //     Commands.deadline(
        //         //new ScoreAlgaeNet(m_drivetrain, m_ElevatorSubsystem, m_AlgaeSubsystem), SCORE ALGI PROCESSOR COMMAND HERE
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.PROCESSOR_RED_POSE.getX()) * MaxSpeed)
        //              .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.PROCESSOR_RED_POSE.getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.PROCESSOR_RED_POSE.getRotation().getDegrees()))
        //         )
        //     )
        // ));

        // // SLOW MODE
        // P1controller.rightTrigger().whileTrue(m_drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
        //         .withVelocityY(-P1controller.getLeftX() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
        //         .withRotationalRate(-P1controller.getRightX() * MaxAngularRate * Constants.DrivetrainConstants.SlowMoSpeed)
        // ));

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
        // P2controller.povLeft().whileTrue(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(2)));
        // P2controller.povRight().whileTrue(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(3)));
        // P2controller.povUp().whileTrue(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(4)));

         P2controller.povDown().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(1)),
         m_ElevatorSubsystem.elevatorToL1(), m_indexing::isP2ManualModeFalse));

         P2controller.povLeft().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(2)),
         m_ElevatorSubsystem.elevatorToL2(), m_indexing::isP2ManualModeFalse));

         P2controller.povRight().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(3)),
         m_ElevatorSubsystem.elevatorToL3(), m_indexing::isP2ManualModeFalse));

         P2controller.povUp().whileTrue(Commands.either(Commands.runOnce(() ->  m_indexing.elevatorIndexChooser(4)),
         m_ElevatorSubsystem.elevatorToL4(), m_indexing::isP2ManualModeFalse));


                
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


        // // Send values to P1

        // P2controller.a().whileTrue(Commands.runOnce(() -> m_indexing.updateP1Index()));

        // Stop Elevator

        P2controller.start().whileTrue(m_ElevatorSubsystem.disableElevator());
        P2controller.back().toggleOnTrue(Commands.startEnd(() -> m_indexing.setP2ManualModeYes(), () -> m_indexing.setP2ManualModeNo(), m_indexing));
        

        P2controller.y().whileTrue(m_ElevatorSubsystem.elevatorToHome());

        //P2controller.b().whileTrue(m_ShooterBoxx.RunShooter(shooterBoxxContants.kSuckSpeed)).onFalse(m_ShooterBoxx.RunShooter(0));
        //P2controller.b().whileTrue(m_ShooterBoxx.IntakeDefaultCommand()).onFalse(m_ShooterBoxx.StopShooterMotor());

        // P2controller.b().whileTrue(
        //         m_ShooterBoxx.BanditSetIntakeMotorCommand(Constants.shooterBoxxContants.kSuckSpeed).until(m_ShooterBoxx::BanditNoteSensorTriggered)
        //       );
        P2controller.b().whileTrue(m_ShooterBoxx.WorksShootCommand());
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
        P3controller.x().whileTrue(m_ShooterBoxx.SpitTillSensorCommand());

        
        // Elevator
        //P3controller.back().whileTrue(m_ElevatorSubsystem.elevatorJoystick(-.5)); //Not sure if this will work needs testing
        // P3controller.povDown().whileTrue(m_ElevatorSubsystem.elevatorToL1());
        // P3controller.povLeft().whileTrue(m_ElevatorSubsystem.elevatorToL2());
        // P3controller.povRight().whileTrue(m_ElevatorSubsystem.elevatorToL3());
        // P3controller.povUp().whileTrue(m_ElevatorSubsystem.elevatorToL4());
        P3controller.povDown().whileTrue(m_AlgaeSuperSystem.ClawGoesForAlgaeCommand());

        P3controller.a().whileTrue(new ScoreCoralAuton( m_ElevatorSubsystem, m_ShooterBoxx,  4).withTimeout(1.75));
        //CLIMB
        
        //P3controller.rightTrigger
        //().whileTrue(m_ClimbSubsystem.ClimbUp());
        //P3controller.leftTrigger().whileTrue(m_ClimbSubsystem.ClimbToHome());
        //P3controller.start().whileTrue(m_ClimbSubsystem.ClimbLockIn());

        //P3controller.leftTrigger().whileTrue(m_ShooterBoxx.SuckTillSensor()).onFalse(m_ShooterBoxx.StopShooterMotor());
        //P3controller.rightTrigger().whileTrue(m_ShooterBoxx.SpitTillSensor()).onFalse(m_ShooterBoxx.StopShooterMotor());
        
        //----------------------------------------------------------P4 Controls-------------------------------------------------------
    
        // //Algae Controls
        // P4controller.leftTrigger().whileTrue(m_AlgaeSubsystem.RegurgitateAlgaeCommand());
        // P4controller.rightTrigger().whileTrue(m_AlgaeSubsystem.IntakeAlgaeCommmand());
        // P4controller.a().whileTrue(
        //     Commands.run(() -> m_AlgaeSuperSystem.ClawGoesForAlgaeOffReef(true), m_AlgaeSuperSystem)
        //     .andThen(m_drivetrain.applyRequest(() ->
        //         driveCentric.withVelocityX(-P1controller.getLeftY() * -Constants.DrivetrainConstants.SlowMoSpeed)
        //         .withVelocityY(-P1controller.getLeftX()) 
        //         .withRotationalRate(-P1controller.getRightX())
        //     ))
        //     .withTimeout(1)
        //     .andThen(m_ElevatorSubsystem.elevatorToL1())
        //     .andThen(() -> m_AlgaeSuperSystem.ClawPlaysNet())
        // );
    
        //P4controller.getRightY((Commands.run(()-> m_AlgaeSubsystem.algaeJoystick(MathUtil.applyDeadband(P4controller.getRightY(), 0.1))));)
    

    // private void updateEnumSmartDashboard(String enumString) {
    //     SmartDashboard.putBoolean("SCORE", false);
    //     SmartDashboard.putBoolean("LEFT_SOURCE", false);
    //     SmartDashboard.putBoolean("RIGHT_SOURCE", false);
    //     SmartDashboard.putBoolean("NET", false);
    //     SmartDashboard.putBoolean("PROCESSOR", false);
    //     SmartDashboard.putBoolean("CLIMB", false);
    //     switch (enumString) {
    //         case "SCORE":
    //             SmartDashboard.putBoolean("SCORE", true);
    //             break;
    //         case "LEFT_SOURCE":
    //             SmartDashboard.putBoolean("LEFT_SOURCE", true);
    //             break;
    //         case "RIGHT_SOURCE":
    //             SmartDashboard.putBoolean("RIGHT_SOURCE", true);
    //             break;
    //         case "NET":
    //             SmartDashboard.putBoolean("NET", true);
    //             break;
    //         case "PROCESSOR":
    //             SmartDashboard.putBoolean("PROCESSOR", true);
    //             break;
    //         case "CLIMB":
    //             SmartDashboard.putBoolean("CLIMB", true);
    //             break;
    //         default:
    //             break;
    //     }
    // }
    //------------------------------------------P1 CONTROLS OFFICIAL FOR TROY-------------------------------------------
    
    // LOCK THE WHEELS
    P1controller.povLeft().whileTrue(m_drivetrain.applyRequest(() -> lockWheels));

    // Slow Mode
    P1controller.rightTrigger().whileTrue(m_drivetrain.applyRequest(() ->
    drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
    .withVelocityY(-P1controller.getLeftX() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
    .withRotationalRate(-P1controller.getRightX() * MaxAngularRate * Constants.DrivetrainConstants.SlowMoSpeed)
    ));

    // REEF SCORING BLUE
        P1controller.leftTrigger().and(m_drivetrain::isAllianceBlue).and(() ->stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsBlue().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))
                
                
                .andThen( m_drivetrain.pathFindToLeftBlueCoralStation().andThen(
                    
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
                )))
            )
        );


        P1controller.leftTrigger().and(m_drivetrain::isAllianceBlue).and(() ->!stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsBlue().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))
                
                
                .andThen( m_drivetrain.pathFindToRightBlueCoralStation().andThen(
                    
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
                )))
            )
        );

        P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).and(() ->stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsRed().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))
                
                
                .andThen( m_drivetrain.pathFindToLeftRedCoralStation().andThen(
                    
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
                )))
            )
        );


        P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).and(() ->!stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsBlue().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))
                
                
                .andThen( m_drivetrain.pathFindToRightRedCoralStation().andThen(
                    
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                        
                    
                )))
            )
        );
        // REEF SCORING RED
        P1controller.a().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToAllTheReefsRed().andThen(
            
                
        m_drivetrain.applyRequest(() ->
            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        
            )
        ));

        P1controller.a().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToAllTheReefsBlue().andThen(
   
        
        m_drivetrain.applyRequest(() ->
            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        
            )
        ));
      
    //------------------------------------------P2 CONTROLS OFFICIAL FOR TROY-------------------------------------------
    
    // Indexing LOL!!
    m_indexing.setDefaultCommand(m_indexing.SettingReefIndexBasedOnController(P2controller::getRightX, P2controller::getRightY));

    // Elevator 

    // Climb

    // Algae
    P4controller.leftTrigger().whileTrue(m_AlgaeSubsystem.RegurgitateAlgaeCommand());
    P4controller.rightTrigger().whileTrue(m_AlgaeSubsystem.IntakeAlgaeCommmand());
    P4controller.a().whileTrue(
        Commands.run(() -> m_AlgaeSuperSystem.ClawGoesForAlgaeOffReef(true), m_AlgaeSuperSystem)
        .andThen(m_drivetrain.applyRequest(() ->
            driveCentric.withVelocityX(-P1controller.getLeftY() * -Constants.DrivetrainConstants.SlowMoSpeed)
            .withVelocityY(-P1controller.getLeftX()) 
            .withRotationalRate(-P1controller.getRightX())
        ))
        .withTimeout(1)
        .andThen(m_ElevatorSubsystem.elevatorToL1())
        .andThen(() -> m_AlgaeSuperSystem.ClawPlaysNet())
    );
    
    //P4controller.getRightY((Commands.run(()-> m_AlgaeSubsystem.algaeJoystick(MathUtil.applyDeadband(P4controller.getRightY(), 0.1))));)
    
    
    // Coral Shooter
    
    
    // Send values to P1

    P2controller.a().whileTrue(Commands.runOnce(() -> m_indexing.updateP1Index()));

    
    
    
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return new AwesomestAutoBlue(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx);
        
    }
}