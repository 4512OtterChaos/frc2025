package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.TunableNumber;

import static frc.robot.subsystems.drivetrain.DriveConstants.*;



/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Represents the linear velocity 100% controller input should currently translate to. Non-functional. */
    public LinearVelocity driveSpeed = MetersPerSecond.of(kDriveSpeed);
    /** Represents the angular velocity 100% controller input should currently translate to. Non-functional. */
    public AngularVelocity turnSpeed = RadiansPerSecond.of(kTurnSpeed);

    private ChassisSpeeds lastTargetSpeeds = new ChassisSpeeds();
    public final SwerveDriveAccelLimiter limiter = new SwerveDriveAccelLimiter(
        kLinearAccel,
        kLinearDecel,
        kAngularAccel,
        kAngularDecel
    );

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    private final PIDController pathXController = new PIDController(kPathDriveKP, kPathDriveKI, kPathDriveKD);
    private final PIDController pathYController = new PIDController(kPathDriveKP, kPathDriveKI, kPathDriveKD);
    private TrapezoidProfile.Constraints pathDriveConstraints = new TrapezoidProfile.Constraints(kDriveSpeed, kLinearAccel);
    private TrapezoidProfile.State pathDriveLastState = new TrapezoidProfile.State();
    private final PIDController pathThetaController = new PIDController(kPathTurnKP, kPathTurnKI, kPathTurnKD);
    private TrapezoidProfile.Constraints pathTurnConstraints = new TrapezoidProfile.Constraints(kTurnSpeed, kAngularAccel);
    private TrapezoidProfile.State pathTurnLastState = new TrapezoidProfile.State();
    private boolean isAligning = false;
    private boolean isAligned = false;
    private Pose2d lastTargetPose = Pose2d.kZero;
    private ChassisSpeeds lastAlignSetpointSpeeds = new ChassisSpeeds();

    {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        pathXController.setTolerance(kPathDrivePosTol, kPathDriveVelTol);
        pathYController.setTolerance(kPathDrivePosTol, kPathDriveVelTol);
        pathThetaController.setTolerance(kPathTurnPosTol, kPathTurnVelTol);

        // isAligned().onTrue(runOnce(()->isAligned = false));
    }

    private final StructPublisher<Pose2d> goalPosePub = NetworkTableInstance.getDefault().getStructTopic("Swerve/Goal Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> targetPosePub = NetworkTableInstance.getDefault().getStructTopic("Swerve/Target Pose", Pose2d.struct).publish();

    private final TunableNumber pathDriveKP = new TunableNumber("Swerve/pathDriveKP", kPathDriveKP);
    private final TunableNumber pathDriveKI = new TunableNumber("Swerve/pathDriveKI", kPathDriveKI);
    private final TunableNumber pathDriveKD = new TunableNumber("Swerve/pathDriveKD", kPathDriveKD);
    private final TunableNumber pathDrivePosTol = new TunableNumber("Swerve/pathDrivePosTol", kPathDrivePosTol);
    private final TunableNumber pathDriveVelTol = new TunableNumber("Swerve/pathDriveVelTol", kPathDriveVelTol);
    
    private final TunableNumber pathTurnKP = new TunableNumber("Swerve/pathTurnKP", kPathTurnKP);
    private final TunableNumber pathTurnKI = new TunableNumber("Swerve/pathTurnKI", kPathTurnKI);
    private final TunableNumber pathTurnKD = new TunableNumber("Swerve/pathTurnKD", kPathTurnKD);
    private final TunableNumber pathTurnPosTol = new TunableNumber("Swerve/pathTurnPosTol", kPathTurnPosTol);
    private final TunableNumber pathTurnVelTol = new TunableNumber("Swerve/pathTurnVelTol", kPathTurnVelTol);

    private final SwerveDrivePoseEstimator visionEstimator = new SwerveDrivePoseEstimator(
        getKinematics(),
        getState().Pose.getRotation(),
        getState().ModulePositions,
        getState().Pose
    );
    private final StructPublisher<Pose2d> estimatedPosePub = NetworkTableInstance.getDefault().getStructTopic("Swerve/Estimated Pose", Pose2d.struct).publish();

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

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followChoreoPath,
            false,
            this,
            trajLogger
        );
    }

    public Command drive(Supplier<ChassisSpeeds> speedsSupplier) {
        return drive(speedsSupplier, true, true);
    }

    public Command drive(Supplier<ChassisSpeeds> speedsSupplier, boolean fieldCentric, boolean limitAccel) {
        return run(() -> {
            var targetSpeeds = speedsSupplier.get();
            if (!fieldCentric) {
                targetSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(targetSpeeds, getState().Pose.getRotation());
            }
            if (limitAccel) {
                targetSpeeds = limiter.calculate(targetSpeeds, lastTargetSpeeds, Robot.kDefaultPeriod);
            }
            lastTargetSpeeds = targetSpeeds;
            if (fieldCentric) {
                setControl(new SwerveRequest.ApplyFieldSpeeds()
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withSpeeds(targetSpeeds));
            }
            else {
                setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, getState().Pose.getRotation())));
            }
        });
    }

    public Command stop() {
        return runOnce(() -> {
            lastTargetSpeeds = new ChassisSpeeds();
            setControl(new SwerveRequest.ApplyRobotSpeeds());
        });
    }

    // public Command driveFacingAngle(Supplier<ChassisSpeeds> speedsSupplier, boolean limitAccel) {
    //     return run(() -> {
    //         var targetSpeeds = speedsSupplier.get();
    //         if (limitAccel) {
    //             targetSpeeds = limiter.calculate(targetSpeeds, lastTargetSpeeds, Robot.kDefaultPeriod);
    //         }
    //         lastTargetSpeeds = targetSpeeds;


    //     });
    // }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Trigger isAligning() {
        return new Trigger(() -> isAligning);
    }

    public Trigger isAligned() {
        return new Trigger(() -> isAligned);
    }

    public Command driveToPose(Supplier<Pose2d> goalSupplier) {
        return sequence(
            // Reset profiles on init
            runOnce(() -> {
                isAligning = true;
                var actual = getState().Pose;
                var goal = goalSupplier.get();
                var speeds = getState().Speeds;

                // Find current velocity towards goal
                var relative = goal.relativeTo(actual);
                var relTrlVec = relative.getTranslation().toVector();
                double velTowardsGoal = 0;
                if (relTrlVec.norm() > 1e-5) {
                    velTowardsGoal = VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).dot(relTrlVec) / relTrlVec.norm();                }
                pathDriveLastState = new TrapezoidProfile.State(0, velTowardsGoal);

                pathTurnLastState = new TrapezoidProfile.State(actual.getRotation().getRadians(), speeds.omegaRadiansPerSecond);

                lastTargetPose = actual;
                lastAlignSetpointSpeeds = lastTargetSpeeds.plus(new ChassisSpeeds());
                pathXController.reset();
                pathYController.reset();
                pathThetaController.reset();
            }),
            // ..then drive to the pose
            drive(() -> {
                var actual = getState().Pose;
                var goal = goalSupplier.get();
                goalPosePub.set(goal);
    
                // PID control on the PREVIOUS setpoint ("compensate for error from feedforward control in the previous timestep")
                var targetSpeeds = new ChassisSpeeds();
                targetSpeeds.vxMetersPerSecond += pathXController.calculate(
                    actual.getX(), lastTargetPose.getX()
                );
                targetSpeeds.vyMetersPerSecond += pathYController.calculate(
                    actual.getY(), lastTargetPose.getY()
                );
                targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
                    actual.getRotation().getRadians(), lastTargetPose.getRotation().getRadians()
                );
    
                // Profile translational movement towards the goal pose
                pathDriveConstraints = new TrapezoidProfile.Constraints(driveSpeed.in(MetersPerSecond), limiter.linearAcceleration);
                var driveProfile = new TrapezoidProfile(pathDriveConstraints);

                // readjust last setpoint to acount for moving goals
                var trlDiff = goal.getTranslation().minus(lastTargetPose.getTranslation());
                var trlDiffVec = trlDiff.toVector();
                var lastSpeedsVec = VecBuilder.fill(lastAlignSetpointSpeeds.vxMetersPerSecond, lastAlignSetpointSpeeds.vyMetersPerSecond);
                double velTowardsGoal = 0;
                if (trlDiff.getNorm() > 1e-5) {
                    velTowardsGoal = lastSpeedsVec.dot(trlDiffVec) / trlDiffVec.norm();
                }
                
                // find the next setpoint from the last setpoint to the (possibly new) goal
                double driveDist = trlDiff.getNorm();
                pathDriveLastState = driveProfile.calculate(
                    0.02,
                    new TrapezoidProfile.State(0, velTowardsGoal),
                    new TrapezoidProfile.State(driveDist, 0)
                );
                var targetTrl = goal.getTranslation();
                if (trlDiff.getNorm() > 1e-5) {
                    double t = pathDriveLastState.position / driveDist;
                    targetTrl = lastTargetPose.getTranslation().plus(trlDiff.times(t));
                }

                // Profile rotational movement towards the goal pose
                pathTurnConstraints = new TrapezoidProfile.Constraints(turnSpeed.in(RadiansPerSecond), limiter.angularAcceleration);
                var turnProfile = new TrapezoidProfile(pathTurnConstraints);
                // Handle continuous angle wrapping
                double goalRadians = goal.getRotation().getRadians();
                double actualRadians = actual.getRotation().getRadians();
                double errorBound = Math.PI;
                double goalMinDistance =
                    MathUtil.inputModulus(goalRadians - actualRadians, -errorBound, errorBound);
                double setpointMinDistance =
                    MathUtil.inputModulus(pathTurnLastState.position - actualRadians, -errorBound, errorBound);

                goalRadians = goalMinDistance + actualRadians;
                pathTurnLastState.position = setpointMinDistance + actualRadians;
                pathTurnLastState = turnProfile.calculate(0.02, pathTurnLastState, new TrapezoidProfile.State(goalRadians, 0));
    
                // Profiled target pose
                lastTargetPose = new Pose2d(targetTrl.getX(), targetTrl.getY(), Rotation2d.fromRadians(pathTurnLastState.position));
                targetPosePub.set(lastTargetPose);
                
                // Use profile setpoint velocities as feedforward targets
                if (trlDiff.getNorm() > 1e-5) {
                    var targetDriveSpeedsTrl = new Translation2d(pathDriveLastState.velocity, 0).rotateBy(trlDiff.getAngle()); // field-relative
                    lastAlignSetpointSpeeds.vxMetersPerSecond = targetDriveSpeedsTrl.getX();
                    lastAlignSetpointSpeeds.vyMetersPerSecond = targetDriveSpeedsTrl.getY();
                    lastAlignSetpointSpeeds.omegaRadiansPerSecond = pathTurnLastState.velocity;
                    targetSpeeds = targetSpeeds.plus(lastAlignSetpointSpeeds);
                }
    
                return targetSpeeds;
            }, true, false)
            .until(() -> { // finish when goal pose is reached
                boolean atSetpoints = pathXController.atSetpoint() && pathYController.atSetpoint() && pathThetaController.atSetpoint();
                var relative = goalSupplier.get().minus(getState().Pose);
                boolean atGoal = MathUtil.isNear(0, relative.getX(), pathDrivePosTol.get());
                atGoal &= MathUtil.isNear(0, relative.getY(), pathDrivePosTol.get());
                atGoal &= MathUtil.isNear(0, relative.getRotation().getRadians(), pathTurnPosTol.get(), -Math.PI, Math.PI);
                boolean finished = atSetpoints && atGoal;
                isAligned = finished;
                return finished;
            })
            .finallyDo((interrupted)->{
                isAligning = false;
            })
        );
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followChoreoPath(SwerveSample sample) {
        

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            applyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        changeTunable();
        
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        //     DriverStation.getAlliance().ifPresent(allianceColor -> {
        //         setOperatorPerspectiveForward(
        //             allianceColor == Alliance.Red
        //                 ? kRedAlliancePerspectiveRotation
        //                 : kBlueAlliancePerspectiveRotation
        //         );
        //         m_hasAppliedOperatorPerspective = true;
        //     });
        // }

        double phoenixTimeOffset = Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
        var state = getState();
        visionEstimator.updateWithTime(state.Timestamp + phoenixTimeOffset, state.RawHeading, state.ModulePositions);
        estimatedPosePub.set(visionEstimator.getEstimatedPosition());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getVisionEstimatedPose() {
        return visionEstimator.getEstimatedPosition();
    }

    public Optional<Pose2d> sampleVisionEstimatedPoseAt(double timestampSeconds) {
        return visionEstimator.sampleAt(timestampSeconds);
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        visionEstimator.resetPose(pose);
    }

    @Override
    public void resetTranslation(Translation2d translation) {
        super.resetTranslation(translation);
        visionEstimator.resetTranslation(translation);
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        super.resetRotation(rotation);
        visionEstimator.resetRotation(rotation);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        visionEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        visionEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    // private void configureAutoBuilder() {
    //     try {
    //         var config = RobotConfig.fromGUISettings();
    //         AutoBuilder.configure(
    //             () -> getState().Pose,   // Supplier of current robot pose
    //             this::resetPose,         // Consumer for seeding pose against auto
    //             () -> getState().Speeds, // Supplier of current robot speeds
    //             // Consumer of ChassisSpeeds and feedforwards to drive the robot
    //             (speeds, feedforwards) -> setControl(
    //                 m_pathApplyRobotSpeeds.withSpeeds(speeds)
    //                     .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
    //                     .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
    //             ),
    //             new PPHolonomicDriveController(
    //                 // PID constants for translation
    //                 new PIDConstants(4, 0, 0),
    //                 // PID constants for rotation
    //                 new PIDConstants(7, 0, 0.1)
    //             ),
    //             config,
    //             () -> false,
    //             this // Subsystem for requirements
    //         );
    //     } catch (Exception ex) {
    //         DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    //     }
    // }

    private void changeTunable() {
        pathDriveKP.poll();
        pathDriveKI.poll();
        pathDriveKD.poll();
        pathDrivePosTol.poll();
        pathDriveVelTol.poll();
        pathTurnKP.poll();
        pathTurnKI.poll();
        pathTurnKD.poll();
        pathTurnPosTol.poll();
        pathTurnVelTol.poll();

        int hash = hashCode();
        // PID
        if (pathDriveKP.hasChanged(hash) || pathDriveKI.hasChanged(hash) || pathDriveKD.hasChanged(hash)) {
            pathXController.setPID(pathDriveKP.get(), pathDriveKI.get(), pathDriveKD.get());
            pathYController.setPID(pathDriveKP.get(), pathDriveKI.get(), pathDriveKD.get());
        }
        if (pathDrivePosTol.hasChanged(hash) || pathDriveVelTol.hasChanged(hash)) {
            pathXController.setTolerance(pathDrivePosTol.get(), pathDriveVelTol.get());
            pathYController.setTolerance(pathDrivePosTol.get(), pathDriveVelTol.get());
        }
        if (pathTurnKP.hasChanged(hash) || pathTurnKI.hasChanged(hash) || pathTurnKD.hasChanged(hash)) {
            pathThetaController.setPID(pathTurnKP.get(), pathTurnKI.get(), pathTurnKD.get());
        }
        if (pathTurnPosTol.hasChanged(hash) || pathTurnVelTol.hasChanged(hash)) {
            pathThetaController.setTolerance(pathTurnPosTol.get(), pathTurnVelTol.get());
        }
    }
}