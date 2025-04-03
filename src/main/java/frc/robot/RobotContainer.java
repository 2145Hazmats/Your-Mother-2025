// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.shooterBoxxContants;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.commands.AlgaeOff;
import frc.robot.commands.FireCoralAuton;
import frc.robot.commands.PutElevatorUp;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.removeAlgaeAuton;
import frc.robot.commands.ScoreCoralAuton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
//import frc.robot.subsystems.AlgaeSuperSystem;
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
    public final SendableChooser<Integer> firstTeleopScoreChooser;

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
    
    private final XboxController P5controller = new XboxController(4);
    private final XboxController P6controller = new XboxController(5);

    // We need to initialize an object of the camera subsystem, we don't have to use it
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(m_drivetrain);
    
    private ClimbSubsystemNeo m_ClimbSubsystemNeo = new ClimbSubsystemNeo();
    private AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
    private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(m_AlgaeSubsystem);
    private ShooterBoxx m_ShooterBoxx = new ShooterBoxx(m_ElevatorSubsystem);
    //private AlgaeSuperSystem m_AlgaeSuperSystem = new AlgaeSuperSystem(m_ElevatorSubsystem, m_AlgaeSubsystem, m_ShooterBoxx);
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
        
          NamedCommands.registerCommand("Algae2Home", m_AlgaeSubsystem.MoveArmToPointCommand(AlgaeConstants.HomePosition));
          NamedCommands.registerCommand("Algae2GrabPosition", m_AlgaeSubsystem.MoveArmToPointCommand(AlgaeConstants.GrabPosition));
          NamedCommands.registerCommand("AlgaeClear", m_AlgaeSubsystem.MoveArmToPointCommand(AlgaeConstants.ProcessorPosition));

          //NamedCommands.registerCommand("SuckTillCoralSensor", m_ShooterBoxx.SuckTillCoralSensorAuto());
          NamedCommands.registerCommand("SuckTillElevatorSensor", m_ShooterBoxx.SuckTillElevatorSensorAuto().finallyDo(() -> m_ShooterBoxx.StopShooterMethod()));
          NamedCommands.registerCommand("StopShooter", m_ShooterBoxx.StopShooterCommand());

          NamedCommands.registerCommand("SuckTillSensor", m_ShooterBoxx.SuckTillCoralSensorAutoCommand()); //OG Command Depricating soon
          NamedCommands.registerCommand("ShootTillSensor", m_ShooterBoxx.SpitTillSensorCommand());
          NamedCommands.registerCommand("SpitOnFloor", Commands.run(() -> m_ShooterBoxx.RunShooter(shooterBoxxContants.kSpitSpeed), m_ShooterBoxx).withTimeout(2));
          NamedCommands.registerCommand("ShootAsFastAsPossible", Commands.run(() -> m_ShooterBoxx.RunShooter(shooterBoxxContants.kAsFastAsPossibleSped), m_ShooterBoxx).finallyDo(() -> m_ShooterBoxx.RunShooter(0)));
          //NamedCommands.registerCommand("StopCoralFromBeingBad", Commands.run(() -> m_ShooterBoxx.RunShooter(0), m_ShooterBoxx));

          //NamedCommands.registerCommand("AutoL4", new ScoreCoralAuton( m_ElevatorSubsystem, m_ShooterBoxx, 4)); //.withTimeout(2).finallyDo(() -> m_ElevatorSubsystem.elevatorToHome()));
          NamedCommands.registerCommand("AutoL4", new PutElevatorUp(m_ElevatorSubsystem, m_ShooterBoxx, 4));
          NamedCommands.registerCommand("FireL4", new FireCoralAuton(m_ElevatorSubsystem, m_ShooterBoxx, 4));

          
          NamedCommands.registerCommand("ScrapeScoreHigh", new removeAlgaeAuton(m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem, 4, true));
          NamedCommands.registerCommand("ScrapeScoreLow", new removeAlgaeAuton(m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem, 4, false));
          

          //NamedCommands.registerCommand("CoastMode" , Commands.runOnce(() -> m_drivetrain.configNeutralMode(NeutralModeValue.Coast)));
          NamedCommands.registerCommand("CoastMode" , Commands.runOnce(() -> {
            m_ShooterBoxx.RunShooter(0);
            m_drivetrain.configNeutralMode(NeutralModeValue.Coast);
            m_ShooterBoxx.RunShooter(0);
          }));
          NamedCommands.registerCommand("BrakeMode" , Commands.runOnce(() -> m_drivetrain.configNeutralMode(NeutralModeValue.Brake)));
          
          
          //NamedCommands.registerCommand("ReadyToLeaveStation", m_ShooterBoxx.ElevatorCoralSensorTriggered());

         autoChooser = AutoBuilder.buildAutoChooser();
         SmartDashboard.putData("Auto Chooser", autoChooser);


         
         stationChooser = new SendableChooser<>();
         stationChooser.setDefaultOption("LeftStation", true);
         //stationChooser.addOption("LeftStation", true);
         stationChooser.addOption("RightStation", false);
         SmartDashboard.putData("Station Chooser", stationChooser);

         firstTeleopScoreChooser = new SendableChooser<>();
         firstTeleopScoreChooser.setDefaultOption("0", 0);
         firstTeleopScoreChooser.addOption("1", 1);
         firstTeleopScoreChooser.addOption("2", 2);
         firstTeleopScoreChooser.addOption("3", 3);
         firstTeleopScoreChooser.addOption("4", 4);
         firstTeleopScoreChooser.addOption("5", 5);
         firstTeleopScoreChooser.addOption("6", 6);
         firstTeleopScoreChooser.addOption("7", 7);
         firstTeleopScoreChooser.addOption("8", 8);
         firstTeleopScoreChooser.addOption("9", 9);
         firstTeleopScoreChooser.addOption("10", 10);
         firstTeleopScoreChooser.addOption("11", 11);
         SmartDashboard.putData("FirstScore Chooser", firstTeleopScoreChooser);
    }

    public CommandSwerveDrivetrain getSwerveDrivetrain() { // for coasting out of auto
        return m_drivetrain;
    }

    public ShooterBoxx getShooterBoxx() { // for coasting out of auto
        return m_ShooterBoxx;
    }

    public void setFirstSpot() {
        m_indexing.firstTeleopPieceChoice(firstTeleopScoreChooser.getSelected());
    }


    private void configureBindings() {
          // Default Commands :)
        //m_ElevatorSubsystem.setDefaultCommand(Commands.either(m_ElevatorSubsystem.defaultCommand(),Commands.run(()-> m_ElevatorSubsystem.elevatorJoystick(P2controller.getLeftY()), m_ElevatorSubsystem) , m_indexing::isP2ManualModeFalse)); //NEEDS TESTING
          //m_ElevatorSubsystem.setDefaultCommand(Commands.either(m_ElevatorSubsystem.elevatorToL1(),m_ElevatorSubsystem.defaultCommand() , m_indexing::isP2ManualModeFalse));
        m_ShooterBoxx.setDefaultCommand(Commands.either(m_ShooterBoxx.IntakeSolosDefaultCommand(), Commands.run(() -> m_ShooterBoxx.StopShooterMethod(), m_ShooterBoxx), m_indexing::isP2ManualModeFalse));
        m_ClimbSubsystemNeo.setDefaultCommand(m_ClimbSubsystemNeo.KeepClimbSafeDefaultCommand());
         //m_AlgaeSubsystem.setDefaultCommand(Commands.run(() -> m_AlgaeSubsystem.algaeJoystick(P2controller.getRightY()), m_AlgaeSubsystem));//(MathUtil.applyDeadband(P4controller.getRightY(), 0.1)), MathUtil.applyDeadband(P4controller.getLeftY(), 0.1)));
        //m_AlgaeSubsystem.setDefaultCommand(m_AlgaeSubsystem.AlgaeDefaultCommand());
          m_indexing.setDefaultCommand(m_indexing.SettingReefIndexBasedOnController(P2controller::getRightX, P2controller::getRightY));
        //m_ShooterBoxx.setDefaultCommand(Commands.run(() -> m_ShooterBoxx.RunShooter(P2controller.getRightX())));
        m_drivetrain.registerTelemetry(logger::telemeterize);

        m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1controller.getLeftX() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-P1controller.getRightX() * MaxAngularRate * m_ElevatorSubsystem.getElevatorSlowSpeed()) // Drive counterclockwise with negative X (left)
            )
        );

        //-------------------------------P1 Controls---------------------------------

        //----new Controllers------- 1 and 2 + 4 and 5 output same joystick direction.getrawaxis
        //for bonus buttons do Trigger testbutton = nwe trigger( -> controller.getrawbutton(6))
        
        //testbutton.whiletrue
        // (() -> P5controller.getRightX()); for sticks ???

        // Trigger plusButton = new Trigger (() -> P5controller.getRawButton(12));
        // Trigger minusButton = new Trigger (() -> P5controller.getRawButton(11));

        // Trigger rightBaby = new Trigger (() -> P5controller.getRawButton(6));
        // Trigger leftBaby = new Trigger (() -> P5controller.getRawButton(3));

        // Trigger leftTrigger = new Trigger (() -> P5controller.getRawButton(9));
        // Trigger rightTrigger = new Trigger (() -> P5controller.getRawButton(10));

        // Trigger rightBumper = new Trigger (() -> P5controller.getRawButton(8));
        // Trigger leftBumper = new Trigger (() -> P5controller.getRawButton(7));


        // rightBaby.onTrue(Commands.runOnce(() -> m_indexing.poseIndexSwitch(false)));

        // P5controller.getRawAxis(3);
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

        // SCORE BLUE LEFT CORAL STATION
        P1controller.leftTrigger().and(m_drivetrain::isAllianceBlue).and(() -> stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsBlue2().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))
                
                // // keep stick out
                // .andThen(Commands.parallel(
                //     Commands.run(() -> m_AlgaeSubsystem.MoveArmToPointMethod(AlgaeConstants.DealgifyPosition)),
                //     Commands.run(() -> m_drivetrain.applyRequest(() -> driveCentric.withVelocityX(DrivetrainConstants.ESCAPE_SPEED).withVelocityY(0).withRotationalRate(0))
                // )).withTimeout(DrivetrainConstants.ESCAPE_TIME).until(m_AlgaeSubsystem::isAlgaeAtHome)) //this instantly ends the backing up
                    
                //.and then(Commands. deadline ) (check if stick is out if so then back up) just use robot centric back up for withTimeout
                .andThen( m_drivetrain.pathFindToLeftBlueCoralStation().andThen(
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                )))
            )
        );

        // SCORE BLUE RIGHT CORAL STATION
        P1controller.leftTrigger().and(m_drivetrain::isAllianceBlue).and(() -> !stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsBlue2().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))

                // // keep stick out
                // .andThen(Commands.parallel(
                //     Commands.run(() -> m_AlgaeSubsystem.MoveArmToPointMethod(AlgaeConstants.DealgifyPosition)),
                //     Commands.run(() -> m_drivetrain.applyRequest(() -> driveCentric.withVelocityX(DrivetrainConstants.ESCAPE_SPEED).withVelocityY(0).withRotationalRate(0))
                // )).withTimeout(DrivetrainConstants.ESCAPE_TIME).until(m_AlgaeSubsystem::isAlgaeAtHome)) //this instantly ends the backing up
                    
                .andThen( m_drivetrain.pathFindToRightBlueCoralStation().andThen(
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_BLUE_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                )))
            )
        );

        // SCORE RED LEFT CORAL STATION
        P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).and(() -> stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsRed2().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))

                // // keep stick out
                // .andThen(Commands.parallel(
                //     Commands.run(() -> m_AlgaeSubsystem.MoveArmToPointMethod(AlgaeConstants.DealgifyPosition)),
                //     Commands.run(() -> m_drivetrain.applyRequest(() -> driveCentric.withVelocityX(DrivetrainConstants.ESCAPE_SPEED).withVelocityY(0).withRotationalRate(0))
                // )).withTimeout(DrivetrainConstants.ESCAPE_TIME).until(m_AlgaeSubsystem::isAlgaeAtHome)) //this instantly ends the backing up
                    
                .andThen( m_drivetrain.pathFindToLeftRedCoralStation().andThen(
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_LEFT_RED_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                )))
            )
        );

        // SCORE RED RIGHT CORAL STATION
        P1controller.leftTrigger().and(m_drivetrain::isAllianceRed).and(() -> !stationChooser.getSelected()).whileTrue(Commands.repeatingSequence(
            //m_indexing.updateP1IndexInRepeatingCommand(),
            m_drivetrain.pathFindToAllTheReefsRed2().andThen(
            Commands.deadline(
                new ScoreCoral(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                ))

                // // keep stick out
                // .andThen(Commands.parallel(
                //     Commands.run(() -> m_AlgaeSubsystem.MoveArmToPointMethod(AlgaeConstants.DealgifyPosition)),
                //     Commands.run(() -> m_drivetrain.applyRequest(() -> driveCentric.withVelocityX(DrivetrainConstants.ESCAPE_SPEED).withVelocityY(0).withRotationalRate(0))
                // )).withTimeout(DrivetrainConstants.ESCAPE_TIME).until(m_AlgaeSubsystem::isAlgaeAtHome)) //this instantly ends the backing up
                    
                .andThen(m_drivetrain.pathFindToRightRedCoralStation().andThen(
                        m_drivetrain.applyRequest(() ->
                            drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getX()) * MaxSpeed)
                            .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getY()) * MaxSpeed)
                            .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CORAL_STATION_RIGHT_RED_POSE.getRotation().getDegrees()))
                         ).until(m_ShooterBoxx::ElevatorCoralSensorTriggered)
                )))
            )
        );
        
        // // GO TO REEF RED NO ARM
        // P1controller.a().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToAllTheReefsRed().andThen(
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //     )
        // ));

        // // GO TO REEF BLUE NO ARM
        // P1controller.a().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToAllTheReefsBlue().andThen(
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //     )
        // ));

        //P1controller.a().whileTrue()

        // PATHFIND AND PID TO BLUE NET CAGE
        // P1controller.povLeft().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToBlueClimbNet().andThen(
        //         m_drivetrain.applyRequest(() ->
        //             drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.CLIMB_BLUE_NET_POSE.getX()) * MaxSpeed)
        //             .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.CLIMB_BLUE_NET_POSE.getY()) * MaxSpeed)
        //             .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.CLIMB_BLUE_NET_POSE.getRotation().getDegrees()))
        //         )  
        // ));

        // Blue
        // P1controller.povLeft().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToBlueClimbNet());
        // P1controller.povRight().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToBlueClimbProcessor());
        // P1controller.povUp().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToBlueClimbCenter());

        // Red
        // P1controller.povLeft().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToRedClimbNet());
        // P1controller.povRight().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToRedClimbProcessor());
        // P1controller.povUp().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToRedClimbCenter());

        // RED
        P1controller.povDown().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToAllTheReefsRed2().andThen(
            Commands.deadline(
                new AlgaeOff(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                )
            )
            .andThen((m_drivetrain.applyRequest(() ->
                driveCentric.withVelocityX(DrivetrainConstants.ESCAPE_SPEED)
                .withVelocityY(0)
                .withRotationalRate(0)
            )
            .withTimeout(DrivetrainConstants.ESCAPE_TIME)
            .until(m_AlgaeSubsystem::isAlgaeAtHome)
            ))
        ));

        // BLUE
        P1controller.povDown().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToAllTheReefsBlue2().andThen(
            Commands.deadline(
                new AlgaeOff(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem), 
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
                    .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
                    .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
                )
            )
            .andThen((m_drivetrain.applyRequest(() ->
                driveCentric.withVelocityX(DrivetrainConstants.ESCAPE_SPEED)
                .withVelocityY(0)
                .withRotationalRate(0)
            )
            .withTimeout(DrivetrainConstants.ESCAPE_TIME)
            .until(m_AlgaeSubsystem::isAlgaeAtHome)
            ))
        ));
             
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

        // FACE Climb
        P1controller.y().whileTrue(m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withVelocityY(-P1controller.getLeftX() * MaxSpeed * m_ElevatorSubsystem.getElevatorSlowSpeed())
                .withRotationalRate(-m_drivetrain.angularSpeedToFaceNet() * m_ElevatorSubsystem.getElevatorSlowSpeed())
        ));

        //CENTRIC MODE
        P1controller.a().whileTrue(m_drivetrain.applyRequest(() ->
        driveCentric.withVelocityX(P1controller.getLeftY() * Constants.DrivetrainConstants.SlowMoSpeed) // Drive forward with negative Y (forward)
             .withVelocityY(P1controller.getLeftX() * Constants.DrivetrainConstants.SlowMoSpeed) // Drive left with negative X (left)
             .withRotationalRate(-P1controller.getRightX() * Constants.DrivetrainConstants.SlowMoSpeed) // Faces the Reef
        ));

        //-------------------------------P2 Controls---------------------------------
        

         P2controller.povDown().whileTrue(Commands.either(Commands.run(() ->  m_indexing.elevatorIndexChooser(1)),
         m_ElevatorSubsystem.elevatorToL1(), m_indexing::isP2ManualModeFalse));

         P2controller.povLeft().whileTrue(Commands.either(Commands.run(() ->  m_indexing.elevatorIndexChooser(2)),
         m_ElevatorSubsystem.elevatorToL2(), m_indexing::isP2ManualModeFalse));

         P2controller.povRight().whileTrue(Commands.either(Commands.run(() ->  m_indexing.elevatorIndexChooser(3)),
         m_ElevatorSubsystem.elevatorToL3(), m_indexing::isP2ManualModeFalse));

         P2controller.povUp().whileTrue(Commands.either(Commands.run(() ->  m_indexing.elevatorIndexChooser(4)),
         m_ElevatorSubsystem.elevatorToL4(), m_indexing::isP2ManualModeFalse));
    
        // Reef Index
        P2controller.leftBumper().onTrue(Commands.runOnce(() -> m_indexing.poseIndexSwitch(false)));
        P2controller.rightBumper().onTrue(Commands.runOnce(() -> m_indexing.poseIndexSwitch(true)));

        // Climb using Neo

        P2controller.leftTrigger().whileTrue(Commands.run(() ->m_ClimbSubsystemNeo.goodClimbInCommand(), m_ClimbSubsystemNeo));
        //P2controller.rightTrigger().whileTrue(Commands.run(() -> m_ClimbSubsystemNeo.PutTheServoInTheRightSpotPlease(), m_ClimbSubsystemNeo).until(()->m_ClimbSubsystemNeo.ReadyToStickTheClimbOutIGuess()).andThen(Commands.waitSeconds(4)).andThen(()-> m_ClimbSubsystemNeo.climbForwardCommand()));//.andThen(() ->m_ClimbSubsystemNeo.climbForwardCommand()));
        //P2controller.x().whileTrue(Commands.run(() ->m_ClimbSubsystemNeo.PutTheServoInTheRightSpotPlease(), m_ClimbSubsystemNeo));
        P2controller.rightTrigger().whileTrue(
            Commands.run(() ->m_ClimbSubsystemNeo.climbOutCommandpart1(), m_ClimbSubsystemNeo)
            .withTimeout(0.5).andThen(Commands.run(() ->m_ClimbSubsystemNeo.climbToSetpointPID(), m_ClimbSubsystemNeo))
        );


        // Send values to P1
        P2controller.a().whileTrue(Commands.runOnce(() -> m_indexing.updateP1Index()));

        // Stop Elevator

        P2controller.start().whileTrue(m_ElevatorSubsystem.disableElevator());
        P2controller.back().toggleOnTrue(Commands.startEnd(() -> m_indexing.setP2ManualModeYes(), () -> m_indexing.setP2ManualModeNo(), m_indexing));

        P2controller.b().whileTrue(m_ShooterBoxx.WorksShootCommand());

        //P2controller.y().whileTrue(m_ElevatorSubsystem.elevatorToHome());
        P2controller.y().whileTrue(Commands.either(Commands.run(() -> m_indexing.updateP1IndexAlgaeEdition() ),
        m_ElevatorSubsystem.elevatorToHome(), m_indexing::isP2ManualModeFalse)); //NOT SURE HOW THIS CALLS DEALGIFY
        
        // Algae stufs
        P2controller.x().whileTrue(m_AlgaeSubsystem.MoveArmToUnJam());
        
        //----------------------------------------------------------P3 Controls-------------------------------------------------------
       
        P3controller.y().whileTrue(
            Commands.run(() ->m_ClimbSubsystemNeo.climbOutCommandpart1(), m_ClimbSubsystemNeo)
            .withTimeout(0.5).andThen(Commands.run(() ->m_ClimbSubsystemNeo.climbToSetpointPID(), m_ClimbSubsystemNeo))
        );
        P3controller.b().whileTrue(Commands.run(() -> m_ClimbSubsystemNeo.climbToNailItPID(), m_ClimbSubsystemNeo));

        P3controller.a().whileTrue(new ScoreCoralAuton( m_ElevatorSubsystem, m_ShooterBoxx,  4).withTimeout(1.75));

        P3controller.leftTrigger().whileTrue(Commands.run(() ->m_ClimbSubsystemNeo.climbInCommand(), m_ClimbSubsystemNeo));
        P3controller.rightTrigger().whileTrue(
            Commands.run(() ->m_ClimbSubsystemNeo.climbOutCommandpart1(), m_ClimbSubsystemNeo)
            .withTimeout(0.5).andThen(Commands.run(() ->m_ClimbSubsystemNeo.climbOutCommandpart2(), m_ClimbSubsystemNeo))
        );

        P3controller.back().whileTrue(Commands.runOnce(() -> m_ClimbSubsystemNeo.resetMotorPosition()));

         
        //----------------------------------------------------------P4 Controls-------------------------------------------------------
    
        //Algae Controls
        P4controller.x().whileTrue( //intake off ground
            Commands.deadline(
                Commands.startEnd(() -> m_AlgaeSubsystem.MoveArmToPointMethodWithSpinner(AlgaeConstants.FloorPosition, AlgaeConstants.intakeSpeed),()-> m_AlgaeSubsystem.stopSpinner(), m_AlgaeSubsystem).until(m_AlgaeSubsystem::AlgaeSensorTriggered),
                Commands.run(() -> m_ElevatorSubsystem.elevatorToPosition(Constants.elevatorConstants.GROUND_INTAKE_HEIGHT), m_ElevatorSubsystem)
            ) );

        P4controller.a().whileTrue( //score L1
        Commands.parallel(
            Commands.run(() -> m_ElevatorSubsystem.elevatorToPosition(Constants.elevatorConstants.L1GROUND_INTAKE_HEIGHT), m_ElevatorSubsystem), 
        
            Commands.startEnd(() -> m_AlgaeSubsystem.MoveArmToPointMethodWithSpinner(AlgaeConstants.ScoreL1Position, 0),()-> m_AlgaeSubsystem.MoveArmToPointMethodWithSpinner(AlgaeConstants.ScoreL1Position, 0), m_AlgaeSubsystem)));
                //(() -> m_AlgaeSubsystem.MoveArmToPointMethodWithSpinner(AlgaeConstants.ScoreL1Position, 0), m_AlgaeSubsystem));
        
        P4controller.povLeft().whileTrue(Commands.run(() -> m_AlgaeSubsystem.MoveArmToPointMethodWithSpinner(AlgaeConstants.ScoreL1Position,Constants.AlgaeConstants.outtakeSpeed)));
            // P4controller.x().whileTrue( //manuel intake off ground
            
            //     Commands.run(() -> m_AlgaeSubsystem.MoveArmToPointMethodWithSpinner(AlgaeConstants.FloorPosition, AlgaeConstants.intakeSpeed), m_AlgaeSubsystem)  );//.until(m_AlgaeSubsystem::AlgaeSensorTriggered));
        P4controller.b().whileTrue(m_AlgaeSubsystem.SpitCoralOnDirtyFloor());
        P4controller.y().whileTrue(m_AlgaeSubsystem.SuckCoralOffDirtyFloor());

        P4controller.povDown().whileTrue(Commands.run(() -> m_AlgaeSubsystem.algaeJoystick(P4controller.getRightY()), m_AlgaeSubsystem).alongWith(Commands.run(()-> m_ElevatorSubsystem.elevatorJoystick(P4controller.getLeftY()), m_ElevatorSubsystem)));
        P4controller.povUp().whileTrue(m_AlgaeSubsystem.AlgaeDefaultCommand());
        //m_AlgaeSubsystem.setDefaultCommand(Commands.run(() -> m_AlgaeSubsystem.algaeJoystick(P2controller.getRightY()), m_AlgaeSubsystem));//(MathUtil.applyDeadband(P4controller.getRightY(), 0.1)), MathUtil.applyDeadband(P4controller.getLeftY(), 0.1)));
        //m_AlgaeSubsystem.setDefaultCommand(m_AlgaeSubsystem.AlgaeDefaultCommand());
        
            

        // P4controller.a().whileTrue(m_AlgaeSubsystem.MoveArmToPointCommand(AlgaeConstants.FloorPosition));
        // P4controller.b().whileTrue(m_AlgaeSubsystem.MoveArmToPointCommand(AlgaeConstants.GrabPosition));
        // P4controller.y().whileTrue(m_AlgaeSubsystem.MoveArmToPointCommand(AlgaeConstants.NetPosition));

        // P4controller.y().whileTrue(Commands.deadline(
        //     new AlgaeOff(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx, m_AlgaeSubsystem),
        //     m_drivetrain.applyRequest(() ->
        //         drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //         .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //         .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        //     )
        // ));

        //P4controller.back().whileTrue(m_AlgaeSubsystem.resetAlgaePosition());

        //P4controller.povDown().whileTrue(Commands.run(() -> m_AlgaeSubsystem.algaeJoystick(P4controller.getRightY()), m_AlgaeSubsystem));

        //---------------------------__FEED FORWARD STUFF------------------------

        // P4controller.a().onTrue(Commands.runOnce(() -> m_drivetrain.startLogging()));
        // P4controller.y().onTrue(Commands.runOnce(() -> m_drivetrain.stopLogging()));

        // P4controller.povUp().whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // P4controller.povDown().whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // P4controller.povRight().whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // P4controller.povLeft().whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));
        
    //------------------------------------------P1 CONTROLS OFFICIAL FOR TROY-------------------------------------------
    
    // LOCK THE WHEELS
    // P1controller.povLeft().whileTrue(m_drivetrain.applyRequest(() -> lockWheels));

    // Slow Mode
    // P1controller.rightTrigger().whileTrue(m_drivetrain.applyRequest(() ->
    // drive.withVelocityX(-P1controller.getLeftY() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
    // .withVelocityY(-P1controller.getLeftX() * MaxSpeed * Constants.DrivetrainConstants.SlowMoSpeed)
    // .withRotationalRate(-P1controller.getRightX() * MaxAngularRate * Constants.DrivetrainConstants.SlowMoSpeed)
    // ));

    // REEF SCORING BLUE
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
        // // REEF SCORING RED
        // P1controller.a().and(m_drivetrain::isAllianceRed).whileTrue(m_drivetrain.pathFindToAllTheReefsRed().andThen(
            
                
        // m_drivetrain.applyRequest(() ->
        //     drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //     .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //     .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.RED_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        
        //     )
        // ));

        // P1controller.a().and(m_drivetrain::isAllianceBlue).whileTrue(m_drivetrain.pathFindToAllTheReefsBlue().andThen(
   
        
        // m_drivetrain.applyRequest(() ->
        //     drive.withVelocityX(m_drivetrain.PIDDriveToPointX(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getX()) * MaxSpeed)
        //     .withVelocityY(m_drivetrain.PIDDriveToPointY(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getY()) * MaxSpeed)
        //     .withRotationalRate(m_drivetrain.PIDDriveToPointDEG(PoseConstants.BLUE_REEF_POSES[m_drivetrain.getPlayer1ReefIndex()].getRotation().getDegrees()))
        
        //     )
        // ));
      
        //------------------------------------------P2 CONTROLS OFFICIAL FOR TROY-------------------------------------------
        
        // Indexing LOL!!
        // m_indexing.setDefaultCommand(m_indexing.SettingReefIndexBasedOnController(P2controller::getRightX, P2controller::getRightY));

        // Elevator 

        // Climb

        // Algae
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
        
        
        // Coral Shooter
        
        
        // Send values to P1

        // P2controller.a().whileTrue(Commands.runOnce(() -> m_indexing.updateP1Index()));


    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return new AwesomestAutoBlue(m_drivetrain, m_ElevatorSubsystem, m_ShooterBoxx);
        
    }
}