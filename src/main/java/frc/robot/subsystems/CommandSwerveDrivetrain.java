package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    // private static final double kSimLoopPeriod = 0.005; // 5 ms
    // private Notifier m_simNotifier = null;
    // private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private PathConstraints pathFindingConstraints;

    private final PIDController pidControllerX = new PIDController(1.65, 0, 0);
    private final PIDController pidControllerY = new PIDController(1.65, 0, 0);
    private final PIDController pidControllerRot = new PIDController(0.01, 0, 0);

    private int reefIndex = 0;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
        configureAutoBuilder();
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
    
    private void configureAutoBuilder() {
        RobotConfig config;
        // Applies generic robot-centric ChassisSpeeds to the drivetrain
        final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        // Tries to run the given code below
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> getState().Pose, //this::getPose2d,
                this::resetPose,
                () -> getState().Speeds, //this::getChassisSpeeds,
                // Consumer for robot-centric speeds and feedfowards
                (speeds, feedforwards) -> setControl( 
                    applyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(5, 0, 0), // Translation PID constants
                    new PIDConstants(5, 0, 0) // Rotation PID constants
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
            PathPlannerConstants.NOMINAL_VOLTAGE_VOLTS, false);

        /* Due to the nature of how Java works, the first run of a pathfinding command could have a
        significantly higher delay compared with subsequent runs.
        To help alleviate this issue, you can run a warmup command in the background when code starts.
        DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER */
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

    /**
     * Pathfinds to a given Pose2d.
     * This avoids field elements and uses pathFindingConstraints.
     *
     * @param pose desired Pose2d
     * @return AutoBuilder command
     */
    public Command pathFindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, pathFindingConstraints, 0.0);
    }

    public Command pathFindToReefAB() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[0], pathFindingConstraints, 0.0); }
    public Command pathFindToReefCD() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[1], pathFindingConstraints, 0.0); }
    public Command pathFindToReefEF() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[2], pathFindingConstraints, 0.0); }
    public Command pathFindToReefGH() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[3], pathFindingConstraints, 0.0); }
    public Command pathFindToReefIJ() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[4], pathFindingConstraints, 0.0); }
    public Command pathFindToReefKL() { return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[5], pathFindingConstraints, 0.0); }

    public Command pathFindToAllTheReefs() {
        return AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[0], pathFindingConstraints, 0)
        .unless(() -> reefIndex != 0)
        .andThen(AutoBuilder.pathfindToPose(PoseConstants.BLUE_REEF_SIDE_POSES[1], pathFindingConstraints, 0))
        .onlyIf(() -> reefIndex == 1);
    }

    // Stops the current CommandSwerveDrivetrain command
    public Command stopCommand() {
        return Commands.none();
    }

    /**
     * Returns true if the alliance is red.
     * Returns false if the alliance is blue.
     * Returns false if no alliance is found.
     *
     * @return boolean
     */
    public boolean isAllianceRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    // Command to face towards the reef
    private PIDController pidfacereef = new PIDController(6, 0, 0); // kP * radians
    public double AngularSpeedToFaceReef() {
        double TriangleY = this.getState().Pose.getY() - 4;
        double TriangleX = this.getState().Pose.getX() - 4.5; // For Blue Alliance
        if (isAllianceRed()) { TriangleX = this.getState().Pose.getX() - 13; } // For Red Alliance
        double angle = Math.atan2(TriangleY, TriangleX); // returns radians
        pidfacereef.enableContinuousInput(-Math.PI, Math.PI);
        SmartDashboard.putNumber("ReefCenterSetpoint", angle);
        return pidfacereef.calculate(this.getState().Pose.getRotation().getRadians(), angle); // messes up with angle jumps from [-179] -> [179]
    }

    public void indexSmartDashboardUpdate(int light) {
        // Updates SmartDashboard Numbers
        SmartDashboard.putNumber("Reef Index", (double) reefIndex);
        SmartDashboard.putNumber("Reef Pose2dX", Constants.PoseConstants.BLUE_REEF_POSES[reefIndex].getX());
        SmartDashboard.putNumber("Reef Pose2dY", Constants.PoseConstants.BLUE_REEF_POSES[reefIndex].getY());
        
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

    public void poseIndexSwitch(boolean clockwise){
        if(clockwise == true) {
            if (reefIndex == 0) { reefIndex = 11; }
            else { reefIndex--; }
        }
        else {
            if(reefIndex == 11) { reefIndex = 0; }
            else{ reefIndex++; }
        }

        indexSmartDashboardUpdate(reefIndex);
    }

    private Pose2d getPose2d() {
        return this.getState().Pose;
    }

    private ChassisSpeeds getChassisSpeeds() {
        return this.getState().Speeds;
    }
    
    public double PIDDriveToPointX(double DesiredPoseX) {
        double SpeedsForPose = pidControllerX.calculate(getPose2d().getX(), DesiredPoseX);
        SpeedsForPose = SpeedsForPose + Math.signum(SpeedsForPose) * .016;
        return SpeedsForPose; 
    }

    public double PIDDriveToPointY(double DesiredPoseY) {
        double SpeedsForPose = pidControllerY.calculate(getPose2d().getY(), DesiredPoseY);
        SpeedsForPose = SpeedsForPose + Math.signum(SpeedsForPose)* .016;
        return SpeedsForPose; 
    }

    public double PIDDriveToPointROT(double DesiredPoseRot) {
        pidControllerRot.enableContinuousInput(-180, 180);
        double SpeedsForPose = pidControllerRot.calculate(Math.abs(getPose2d().getRotation().getDegrees()), DesiredPoseRot);
        SpeedsForPose = SpeedsForPose * Math.signum(getPose2d().getRotation().getDegrees());
        SpeedsForPose = SpeedsForPose + Math.signum(SpeedsForPose)* .016;
        return SpeedsForPose;
    }

    @Override
    public void periodic() {
        // Periodically try to apply the operator perspective. If we haven't applied the operator perspective before,
        // then we should apply it regardless of DS state. This allows us to correct the perspective in case the robot
        // code restarts mid-match. Otherwise, only check and apply the operator perspective if the DS is disabled.
        // This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
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

    /* =============== SYSTEM ID METHODS =============== */
    /* =============== SYSTEM ID METHODS =============== */
    /* =============== SYSTEM ID METHODS =============== */

    /* Swerve requests to apply during SysId characterization */
    //private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
   
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    // private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> setControl(m_translationCharacterization.withVolts(output)),
    //         null,
    //         this
    //     )
    // );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(7), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
    //         null,
    //         this
    //     )
    // );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //             /* also log the requested output for SysId */
    //             SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    //         },
    //         null,
    //         this
    //     )
    // );

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.quasistatic(direction);
    // }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.dynamic(direction);
    // }

    /* The SysId routine to test */
    //private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* =============== SIMULATION METHODS =============== */
    /* =============== SIMULATION METHODS =============== */
    /* =============== SIMULATION METHODS =============== */

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
