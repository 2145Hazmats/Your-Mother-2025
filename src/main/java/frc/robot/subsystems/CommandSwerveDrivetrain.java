package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.ReefConstants.ReefMathConstants;
import frc.robot.ReefConstants.PoseConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private PathConstraints pathFindingConstraints;
   // private PathConstraints pathFindingStationConstraints;


    private int player1ReefIndex = 0;
    private int player2ReefIndex = 0;

    //private final ProfiledPIDController pidControllerX = new ProfiledPIDController(DrivetrainConstants.P_X, DrivetrainConstants.I_X, DrivetrainConstants.D_X, new Constraints(DrivetrainConstants.PROFILED_PID_MAX_VELOCITY, DrivetrainConstants.PROFILED_PID_MAX_ACCELERATION));
    //private final ProfiledPIDController pidControllerY = new ProfiledPIDController(DrivetrainConstants.P_X, DrivetrainConstants.I_X, DrivetrainConstants.D_X, new Constraints(DrivetrainConstants.PROFILED_PID_MAX_VELOCITY, DrivetrainConstants.PROFILED_PID_MAX_ACCELERATION));
    private final PIDController pidControllerDeg = new PIDController(DrivetrainConstants.P_DEGREE, 0, DrivetrainConstants.D_DEGREE);
    
    private final PIDController StationpidControllerX = new PIDController(DrivetrainConstants.P_X, DrivetrainConstants.I_X, DrivetrainConstants.D_X);
    private final PIDController StationpidControllerY = new PIDController(DrivetrainConstants.P_X, DrivetrainConstants.I_X, DrivetrainConstants.D_X);
    private final PIDController StationpidControllerDeg = new PIDController(DrivetrainConstants.P_DEGREE, 0, DrivetrainConstants.D_DEGREE);

    private final PIDController pidFaceRad = new PIDController(DrivetrainConstants.PID_RAD, 0, 0); // kP * radians

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        configureAutoBuilder();

        SignalLogger.setPath("/home/lvuser/logs");
        //SignalLogger.start();

        pidFaceRad.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void startLogging() {
        SignalLogger.start();
    }

    public void stopLogging() {
        SignalLogger.stop();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
    
    // Configure All of pathplanner so we can use it :>
    private void configureAutoBuilder() {
        RobotConfig config;
        // Applies generic robot-centric ChassisSpeeds to the drivetrain
        final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        // Tries to run the given code below
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> getPose2d(), 
                this::resetPose,
                () -> getState().Speeds,
                // Consumer for robot-centric speeds and feedfowards
                (speeds, feedforwards) -> setControl( 
                    applyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    PathPlannerConstants.TRANSLATIONAL_PID, // Translation PID constants
                    PathPlannerConstants.ROTATIONAL_PID // Rotation PID constants
                ),
                config,
                // Makes the path mirrored for red alliance. Only set on robot startup
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner YOU SUCK! LOOK AT YOUR COMMAND SWERVE DRIVE CONFIGURE METHOD", e.getStackTrace());
        }

        pathFindingConstraints = new PathConstraints(
            PathPlannerConstants.MAX_VELOCITY_MPS,
            PathPlannerConstants.MAX_ACCELERATION_MPS,
            PathPlannerConstants.MAX_ANGULAR_VELOCITY_RAD,
            PathPlannerConstants.MAX_ANGULAR_ACCELERATION_RAD,
            PathPlannerConstants.NOMINAL_VOLTAGE_VOLTS, 
            false
        );

        // DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER 
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Goes directly to the start of a given PathPlanner path and then follows the path.
     * This does NOT use any pathfinding or field element avoidance.
     *
     * @param pathFileName file name of a PathPlanner path
     * @return AutoBuilder command
     */
    public Command followPath(String pathFileName) {
        try{ 
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathFileName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("FollowPath ERROR", e.getStackTrace());
            return Commands.none();
        }
    }

    /**
     * Pathfinds to the start of a given PathPlanner path and then follows the path.
     * This avoids field elements and uses pathFindingConstraints only during the pathfinding segment (before it reaches the start of the path).
     *
     * @param pathFileName file name of a PathPlanner path
     * @return AutoBuilder command
     */
    public Command pathFindThenFollowPath(String pathFileName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathFileName);
            return AutoBuilder.pathfindThenFollowPath(path, pathFindingConstraints);
        } catch (Exception e) {
            DriverStation.reportError("PathFindThenFollowPath ERROR", e.getStackTrace());
            return Commands.none();
        }
    }

    // Magic Blue Side Reef Pathfinding Button
    public Command pathFindToAllTheReefsBlue() {
        return AutoBuilder.pathfindToPose(
        PoseConstants.BLUE_REEF_SIDE_POSES[0], pathFindingConstraints, 0).onlyIf(() -> player1ReefIndex == 0)
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[0], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 1))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 2))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 3))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 4))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 5))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 6))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 7))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 8))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 9))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 10))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 11));
    }

    // Magic Red Side Reef Pathfinding Button
    public Command pathFindToAllTheReefsRed() {
        return AutoBuilder.pathfindToPose(
        PoseConstants.RED_REEF_SIDE_POSES[0], pathFindingConstraints, 0).onlyIf(() -> player1ReefIndex == 0)
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[0], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 1))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 2))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 3))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 4))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 5))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 6))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 7))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 8))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 9))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 10))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 11));
    }

    // NEW NEW NEW
    // Magic Blue Side Reef Pathfinding Button
    public Command pathFindToAllTheReefsBlue2() {
        return AutoBuilder.pathfindToPose(
        PoseConstants.PATHFIND_BLUE_REEF_POSES[0], pathFindingConstraints, 0).onlyIf(() -> player1ReefIndex == 0)
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 1))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 2))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 3))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 4))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 5))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[6], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 6))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[7], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 7))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[8], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 8))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[9], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 9))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[10], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 10))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_BLUE_REEF_POSES[11], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 11));
    }

    // NEW NEW NEW
    // Magic Red Side Reef Pathfinding Button
    public Command pathFindToAllTheReefsRed2() {
        return AutoBuilder.pathfindToPose(
        PoseConstants.PATHFIND_RED_REEF_POSES[0], pathFindingConstraints, 0).onlyIf(() -> player1ReefIndex == 0)
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 1))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 2))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 3))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 4))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 5))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[6], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 6))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[7], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 7))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[8], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 8))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[9], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 9))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[10], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 10))
        .beforeStarting(AutoBuilder.pathfindToPose(PoseConstants.PATHFIND_RED_REEF_POSES[11], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS).onlyIf(() -> player1ReefIndex == 11));
    }

    // Goes to a certain point instead of all of them
    public Command pathFindToReefBlueAB() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[0], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefBlueCD() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefBlueEF() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefBlueGH() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefBlueIJ() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefBlueKL() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    
    public Command pathFindToReefRedAB() { return AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[0], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefRedCD() { return AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[1], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefRedEF() { return AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[2], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefRedGH() { return AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[3], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefRedIJ() { return AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[4], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToReefRedKL() { return AutoBuilder.pathfindToPose(PoseConstants.RED_REEF_SIDE_POSES[5], pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }

    // Goes to each coral station
    public Command pathFindToLeftBlueCoralStation() {
        return AutoBuilder.pathfindToPose(PoseConstants.CORAL_STATION_LEFT_BLUE_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS_STATION);
    }
    public Command pathFindToRightBlueCoralStation() {
        return AutoBuilder.pathfindToPose(PoseConstants.CORAL_STATION_RIGHT_BLUE_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS_STATION);
    }
    public Command pathFindToLeftRedCoralStation() {
        return AutoBuilder.pathfindToPose(PoseConstants.CORAL_STATION_LEFT_RED_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS_STATION);
    }
    public Command pathFindToRightRedCoralStation() {
        return AutoBuilder.pathfindToPose(PoseConstants.CORAL_STATION_RIGHT_RED_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS_STATION);
    }

    public Command pathFindToRedNet() {
        return AutoBuilder.pathfindToPose(PoseConstants.NET_RED_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS);
    }
    public Command pathFindToBlueNet() {
        return AutoBuilder.pathfindToPose(PoseConstants.NET_BLUE_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS);
    }

    public Command pathFindToBlueProcessor() {
        return AutoBuilder.pathfindToPose(PoseConstants.PROCESSOR_BLUE_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS);
    }

    public Command pathFindToRedProcessor() {
        return AutoBuilder.pathfindToPose(PoseConstants.PROCESSOR_RED_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS);
    }

    // CLIMB BLUE PATHFIND
    public Command pathFindToBlueClimbNet() { return AutoBuilder.pathfindToPose(PoseConstants.CLIMB_BLUE_NET_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToBlueClimbCenter() { return AutoBuilder.pathfindToPose(PoseConstants.CLIMB_BLUE_CENTER_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToBlueClimbProcessor() { return AutoBuilder.pathfindToPose(PoseConstants.CLIMB_BLUE_PROCCESOR_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    // CLIMB RED PATHFIND
    public Command pathFindToRedClimbNet() { return AutoBuilder.pathfindToPose(PoseConstants.CLIMB_RED_NET_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToRedClimbCenter() { return AutoBuilder.pathfindToPose(PoseConstants.CLIMB_RED_CENTER_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }
    public Command pathFindToRedClimbProcessor() { return AutoBuilder.pathfindToPose(PoseConstants.CLIMB_RED_PROCCESOR_PATHFIND_POSE, pathFindingConstraints, PathPlannerConstants.PATHFIND_END_SPEED_MPS); }

    //SPECIAL STATION CODER HERE ------------------------------------------

    public double StationPIDDriveToPointX(double DesiredPoseX) {
        SmartDashboard.putNumber("GOTO PID POSE X", DesiredPoseX);
        double SpeedsForPose = StationpidControllerX.calculate(getPose2d().getX(), DesiredPoseX);
        SpeedsForPose = Math.min(Math.max(SpeedsForPose, -DrivetrainConstants.STATION_PID_MAX), DrivetrainConstants.STATION_PID_MAX);
        if (isAllianceRed()) {
            return -SpeedsForPose;
        }
        return SpeedsForPose; 
    }
    // return speed for the Y Direction to get to desired Pose
    public double StationPIDDriveToPointY(double DesiredPoseY) {
        SmartDashboard.putNumber("GOTO PID POSE Y", DesiredPoseY);
        double SpeedsForPose = StationpidControllerY.calculate(getPose2d().getY(), DesiredPoseY);
        SpeedsForPose = Math.min(Math.max(SpeedsForPose, -DrivetrainConstants.STATION_PID_MAX), DrivetrainConstants.STATION_PID_MAX);
        if (isAllianceRed()) {
            return -SpeedsForPose;
        }
        return SpeedsForPose;
    }

    //------------------------------------------

    // return speed for the X Direction to get to desired Pose
    public double PIDDriveToPointX(double DesiredPoseX) {
        //pidControllerX.reset(getPose2d().getX());
        //double SpeedsForPose = pidControllerX.calculate(getPose2d().getX(), DesiredPoseX);
        double SpeedsForPose = ((DesiredPoseX - getPose2d().getX()) * DrivetrainConstants.FAKE_PID_P) + (DrivetrainConstants.FAKE_PID_FEED_FORWARD * Math.signum(DesiredPoseX - getPose2d().getX()));
        if (isAllianceRed()) {
            return -SpeedsForPose;
        }
        return SpeedsForPose; 
    }

    // return speed for the Y Direction to get to desired Pose
    public double PIDDriveToPointY(double DesiredPoseY) {
        //pidControllerY.reset(getPose2d().getY());
        //double SpeedsForPose = pidControllerY.calculate(getPose2d().getY(), DesiredPoseY);
        double SpeedsForPose = ((DesiredPoseY - getPose2d().getY()) * DrivetrainConstants.FAKE_PID_P) + (DrivetrainConstants.FAKE_PID_FEED_FORWARD * Math.signum(DesiredPoseY - getPose2d().getY()));
        //SpeedsForPose = MathUtil.clamp(SpeedsForPose, -DrivetrainConstants.FAKE_PID_MAX_SPEED, DrivetrainConstants.FAKE_PID_MAX_SPEED);
        if (isAllianceRed()) {
            return -SpeedsForPose;
        }
        return SpeedsForPose;
    }

    // return angular speed to rotate to desired Degrees
    public double PIDDriveToPointDEG(double DesiredPoseDeg) {
        SmartDashboard.putNumber("GOTO PID POSE DEG", DesiredPoseDeg);
        pidControllerDeg.enableContinuousInput(-180, 180);
        double SpeedsForPose = pidControllerDeg.calculate(getPose2d().getRotation().getDegrees(), DesiredPoseDeg);
        //SpeedsForPose = SpeedsForPose * Math.signum(getPose2d().getRotation().getDegrees());
        //SpeedsForPose = SpeedsForPose + Math.signum(SpeedsForPose) * DrivetrainConstants.FEEDFORWARD_CONSTANT_DEGREE;
        // Do something like this to use the onboard PID controller for each motor or to use motionMagic
        //this.getModule(0).getSteerMotor().setControl(new VelocityVoltage(SpeedsForPose));
        return SpeedsForPose;
    }

    // Command to face towards the reef
    public double angularSpeedToFaceReef() {
        double TriangleY = this.getState().Pose.getY() - 4;
        double TriangleX = this.getState().Pose.getX() - 4.5; // For Blue Alliance
        if (isAllianceRed()) { TriangleX = this.getState().Pose.getX() - 13; } // For Red Alliance
        double angle = Math.atan2(TriangleY, TriangleX)+Math.PI; // returns radians

        SmartDashboard.putNumber("ReefCenterSetpoint", angle);
        return -pidFaceRad.calculate(this.getState().Pose.getRotation().getRadians(), angle); 
    }
    // Command to face towards left Coral station
    public double angularSpeedToFaceLeftCoralStation() {
        double angle = PoseConstants.BLUE_CORAL_STATION_DEG; // For Blue Alliance
        if (isAllianceRed()) { angle = -PoseConstants.RED_CORAL_STATION_DEG; } // For Red Alliance
        angle = Units.degreesToRadians(angle); // convert to radians
        return -pidFaceRad.calculate(this.getState().Pose.getRotation().getRadians(), angle);
    }
    // Command to face towards right Coral station
    public double angularSpeedToFaceRightCoralStation() {
        double angle = -PoseConstants.BLUE_CORAL_STATION_DEG; // For Blue Alliance
        if (isAllianceRed()) { angle = PoseConstants.RED_CORAL_STATION_DEG; } // For Red Alliance
        angle = Units.degreesToRadians(angle); // convert to radians
        return -pidFaceRad.calculate(this.getState().Pose.getRotation().getRadians(), angle); 
    }
    // Command to face towards net
    public double angularSpeedToFaceNet() {
        double angle = 0; // For Blue Alliance
        if (isAllianceRed()) { angle = 180; } // For Red Alliance
        angle = Units.degreesToRadians(angle); // convert to radians
        return -pidFaceRad.calculate(this.getState().Pose.getRotation().getRadians(), angle);
    }

    // Switches our reef index
    public void poseIndexSwitch(boolean clockwise){
        if(clockwise == true) {
            if (player2ReefIndex == 0) { player2ReefIndex = 11; }
            else { player2ReefIndex--; }
        }
        else {
            if(player2ReefIndex == 11) { player2ReefIndex = 0; }
            else{ player2ReefIndex++; }
        }

        indexSmartDashboardUpdate(player2ReefIndex);
    }

    public void updateP1Index() {
        player1ReefIndex = player2ReefIndex;
        SmartDashboard.putNumber("player1ReefIndex", (double) player1ReefIndex);
    }

    public void setP1Index(int i) {
        player1ReefIndex = i;
    }

    public void setP2Index(int index) {
        player2ReefIndex = index;
        indexSmartDashboardUpdate(player2ReefIndex);
    }

    public int getPlayer2ReefIndex() {
        return player2ReefIndex;
    }

    
    // Updates SmartDashboard Numbers
    public void indexSmartDashboardUpdate(int light) {
        SmartDashboard.putNumber("player2ReefIndex", (double) player2ReefIndex);
        
        // Resets SmartDashboard Reef Interface Circle
        SmartDashboard.putBoolean("ReefPos0", false); SmartDashboard.putBoolean("ReefPos1", false);
        SmartDashboard.putBoolean("ReefPos2", false); SmartDashboard.putBoolean("ReefPos3", false);
        SmartDashboard.putBoolean("ReefPos4", false); SmartDashboard.putBoolean("ReefPos5", false);
        SmartDashboard.putBoolean("ReefPos6", false); SmartDashboard.putBoolean("ReefPos7", false);
        SmartDashboard.putBoolean("ReefPos8", false); SmartDashboard.putBoolean("ReefPos9", false);
        SmartDashboard.putBoolean("ReefPos10", false); SmartDashboard.putBoolean("ReefPos11", false);

        // Finds our index value and highlights to correct boolean
        if (light == 0) {SmartDashboard.putBoolean("ReefPos0", true);}
        else if (light == 1) {SmartDashboard.putBoolean("ReefPos1", true);}
        else if (light == 2) {SmartDashboard.putBoolean("ReefPos2", true);}
        else if (light == 3) {SmartDashboard.putBoolean("ReefPos3", true);}
        else if (light == 4) {SmartDashboard.putBoolean("ReefPos4", true);}
        else if (light == 5) {SmartDashboard.putBoolean("ReefPos5", true);}
        else if (light == 6) {SmartDashboard.putBoolean("ReefPos6", true);}
        else if (light == 7) {SmartDashboard.putBoolean("ReefPos7", true);}
        else if (light == 8) {SmartDashboard.putBoolean("ReefPos8", true);}
        else if (light == 9) {SmartDashboard.putBoolean("ReefPos9", true);}
        else if (light == 10) {SmartDashboard.putBoolean("ReefPos10", true);}
        else {SmartDashboard.putBoolean("ReefPos11", true);}
    }

    // Stops the current CommandSwerveDrivetrain command
    public Command stopCommand() {
        return Commands.none();
    }

    // Returns true if the alliance is red
    public boolean isAllianceRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public boolean isPoseCloseEnoughToSetpointBlue() {
        return (getPose2d().getX() < PoseConstants.BLUE_REEF_POSES[player1ReefIndex].getX()+Constants.ErrorConstants.DriveTrainScoreError
                && getPose2d().getX() > PoseConstants.BLUE_REEF_POSES[player1ReefIndex].getX()-Constants.ErrorConstants.DriveTrainScoreError
                && getPose2d().getY() < PoseConstants.BLUE_REEF_POSES[player1ReefIndex].getY()+Constants.ErrorConstants.DriveTrainScoreError
                && getPose2d().getY() > PoseConstants.BLUE_REEF_POSES[player1ReefIndex].getY()-Constants.ErrorConstants.DriveTrainScoreError);
    }

    public boolean isPoseCloseEnoughToSetpointRed() {
        return (getPose2d().getX() < PoseConstants.RED_REEF_POSES[player1ReefIndex].getX()+Constants.ErrorConstants.DriveTrainScoreError
            && getPose2d().getX() > PoseConstants.RED_REEF_POSES[player1ReefIndex].getX()-Constants.ErrorConstants.DriveTrainScoreError
            && getPose2d().getY() < PoseConstants.RED_REEF_POSES[player1ReefIndex].getY()+Constants.ErrorConstants.DriveTrainScoreError
            && getPose2d().getY() >PoseConstants.RED_REEF_POSES[player1ReefIndex].getY()-Constants.ErrorConstants.DriveTrainScoreError);
    }

    // Returns true if the alliance is blue
    public boolean isAllianceBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    }

    public Pose2d getPose2d() { //THIS IS THE GOOD ONE FROM STATES
        return this.getState().Pose;
    }

    // public Pose2d getPose2d() { //doesn't currently reset pose just gives methods an illusion
    //     if (this.getState().Pose.getX() > 17.55 && this.getState().Pose.getY() <0) { //bottom right 
    //         return new Pose2d(17.55,0,this.getState().Pose.getRotation());//this.getState().Pose;
    //     }
    //     else if (this.getState().Pose.getX() > 17.55 && this.getState().Pose.getY() >8.05) { // top right
    //         return new Pose2d(17.55,8.05,this.getState().Pose.getRotation());
    //     }
    //     else if (this.getState().Pose.getX() < 0 && this.getState().Pose.getY() <0) { //bottom left
    //         return new Pose2d(0,0,this.getState().Pose.getRotation());
    //     } 
    //     else if (this.getState().Pose.getX() < 0 && this.getState().Pose.getY() >8.05) { //top left
    //         return new Pose2d(0,8.05,this.getState().Pose.getRotation());
    //     }
    //     else if (this.getState().Pose.getX() < 0) {//left jungle
    //         return new Pose2d(0,this.getState().Pose.getY(),this.getState().Pose.getRotation());
    //     }
    //     else if (this.getState().Pose.getX() > 17.55){ //right jungle

    //         return new Pose2d(17.55,this.getState().Pose.getY(),this.getState().Pose.getRotation());
    //     }
    //     else if (this.getState().Pose.getY() > 8.05) { //top jungle

    //         return new Pose2d(this.getState().Pose.getX(),8.05,this.getState().Pose.getRotation());
    //     } else if (this.getState().Pose.getY() <0) { //bottom jungle

    //         return new Pose2d(this.getState().Pose.getX(),0,this.getState().Pose.getRotation());
    //     }

    //     else {
    //         return this.getState().Pose;
    //     }
    // }

    public int getPlayer1ReefIndex() {
        return this.player1ReefIndex;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.getState().Speeds;
    }

    @Override
    public void periodic() {
        if (isAllianceRed()) {
            SmartDashboard.putNumber("Reef Pose X", PoseConstants.RED_REEF_POSES[player1ReefIndex].getX());
            SmartDashboard.putNumber("Reef Pose Y", PoseConstants.RED_REEF_POSES[player1ReefIndex].getY());
            SmartDashboard.putNumber("Reef Pose Deg", PoseConstants.RED_REEF_POSES[player1ReefIndex].getRotation().getDegrees());
        } else {
            SmartDashboard.putNumber("Reef Pose X", PoseConstants.BLUE_REEF_POSES[player1ReefIndex].getX());
            SmartDashboard.putNumber("Reef Pose Y", PoseConstants.BLUE_REEF_POSES[player1ReefIndex].getY());
            SmartDashboard.putNumber("Reef Pose Deg", PoseConstants.BLUE_REEF_POSES[player1ReefIndex].getRotation().getDegrees());
        }
        //CHECK TO SEE WHETEHR OUR CONDITIONS ARE TRUE TO FIRE
    if (isAllianceRed()) {
         if (isPoseCloseEnoughToSetpointRed()) {
            SmartDashboard.putBoolean("Alligned", true);
        } else SmartDashboard.putBoolean("Alligned", false);

    } else { //BLUE POSE CHECKS
         if (isPoseCloseEnoughToSetpointBlue()) {
            SmartDashboard.putBoolean("Alligned", true);
        } else SmartDashboard.putBoolean("Alligned", false);
    } 
        
        //this.resetRotation(newIMU.getRotation2d());

        // Periodically try to apply the operator perspective.
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /* =============== CONSTRUCTORS =============== */
    /* =============== CONSTRUCTORS =============== */
    /* =============== CONSTRUCTORS =============== */

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, modules);
    //     if (Utils.isSimulation()) {
    //         //startSimThread();
    //     }
    //     configureAutoBuilder();
    // }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     Matrix<N3, N1> odometryStandardDeviation,
    //     Matrix<N3, N1> visionStandardDeviation,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    //     if (Utils.isSimulation()) {
    //         //startSimThread();
    //     }
    //     configureAutoBuilder();
    // }

    // Coral Station: 4.45 meters from reef
    // Far Climb: 5.35 meters from reef
    public double getDistanceFromReef() {
        double blueReefPoseX = 4.479;
        double blueReefPoseY = 4.025;
        double redReefPoseX = 13.069;
        double redReefPoseY = 4.025;

        if (isAllianceBlue()) {
            return (Math.sqrt(
                Math.pow(blueReefPoseX - getPose2d().getX(), 2) +
                Math.pow(blueReefPoseY - getPose2d().getY(), 2)
            ));
        } else if (isAllianceRed()) {
            return (Math.sqrt(
                Math.pow(redReefPoseX - getPose2d().getX(), 2) +
                Math.pow(redReefPoseY - getPose2d().getY(), 2)
            ));
        }
        return 0;
    };

    /* =============== SYSTEM ID METHODS =============== */
    /* =============== SYSTEM ID METHODS =============== */
    /* =============== SYSTEM ID METHODS =============== */

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
   
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* =============== SIMULATION METHODS =============== */
    /* =============== SIMULATION METHODS =============== */
    /* =============== SIMULATION METHODS =============== */

    // private static final double kSimLoopPeriod = 0.005; // 5 ms
    // private Notifier m_simNotifier = null;
    // private double m_lastSimTime;

    // private void startSimThread() {
    //     m_lastSimTime = Utils.getCurrentTimeSeconds();
    //     /* Run simulation at a faster rate so PID gains behave more reasonably */
    //     m_simNotifier = new Notifier(() -> {
    //         final double currentTime = Utils.getCurrentTimeSeconds();
    //         double deltaTime = currentTime - m_lastSimTime;
    //         m_lastSimTime = currentTime;
    //         /* use the measured time delta, get battery voltage from WPILib */
    //         updateSimState(deltaTime, RobotController.getBatteryVoltage());
    //     });
    //     m_simNotifier.startPeriodic(kSimLoopPeriod);
    // }
}
