package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TunableNumber;

public class AutoAlign extends Command {

    private final SwerveDriveLimiter limiter;

    private final ProfiledPIDController xController = new ProfiledPIDController(
        kPathDriveKP, kPathDriveKI, kPathDriveKD,
        new TrapezoidProfile.Constraints(0, 0)
    );
    private final ProfiledPIDController yController = new ProfiledPIDController(
        kPathDriveKP, kPathDriveKI, kPathDriveKD,
        new TrapezoidProfile.Constraints(0, 0)
    );
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        kPathTurnKP, kPathTurnKI, kPathTurnKD,
        new TrapezoidProfile.Constraints(0, 0)
    );

    private Pose2d lastSetpointPose;
    private ChassisSpeeds lastSetpointSpeeds;

    private final Supplier<Pose2d> currentPoseSupplier;
    private final Supplier<Pose2d> goalPoseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private final Consumer<ChassisSpeeds> targetFieldSpeedsConsumer;

    // Static tunables
    private static final TunableNumber pathDriveKP = new TunableNumber("Align/pathDriveKP", kPathDriveKP);
    private static final TunableNumber pathDriveKD = new TunableNumber("Align/pathDriveKD", kPathDriveKD);
    private static final TunableNumber pathTurnKP = new TunableNumber("Align/pathTurnKP", kPathTurnKP);
    private static final TunableNumber pathTurnKD = new TunableNumber("Align/pathTurnKD", kPathTurnKD);

    public static class Config {
        /** Limiter for normal alignment speeds */
        public SwerveDriveLimiter standardLimiter = kAlignLimiter.copy();
        /** Limiter for final alignment speeds to goal pose */
        public SwerveDriveLimiter finishLimiter = kAlignFinalLimiter.copy();
        /** Reference swerve limiter. The auto-align speeds will never be more aggressive than this. */
        public SwerveDriveLimiter referenceLimiter;

        public Distance drivePosTol = Meters.of(kAlignDrivePosTol);
        public LinearVelocity driveVelTol = MetersPerSecond.of(kAlignDriveVelTol);
        public Angle thetaPosTol = Radians.of(kAlignTurnPosTol);
        public AngularVelocity thetaVelTol = RadiansPerSecond.of(kAlignTurnVelTol);

        public Distance finalAlignDist = Meters.of(kFinalAlignDist);

        public boolean alignBackwards = false;
    }

    // Instance tunables
    private final Config config;
    private final TunableNumber drivePosTol;
    private final TunableNumber driveVelTol;
    private final TunableNumber thetaPosTol;
    private final TunableNumber thetaVelTol;
    private final TunableNumber finalAlignDist;

    private final StructPublisher<Pose2d> goalPosePub;
    private final StructPublisher<Pose2d> targetPosePub;

    public AutoAlign(
            String name,
            Supplier<Pose2d> currentPoseSupplier, Supplier<Pose2d> goalPoseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier, Consumer<ChassisSpeeds> targetFieldSpeedsConsumer)
    {
        this(name, currentPoseSupplier, goalPoseSupplier, fieldSpeedsSupplier, targetFieldSpeedsConsumer, new Config());
    }

    public AutoAlign(
            String name,
            Supplier<Pose2d> currentPoseSupplier, Supplier<Pose2d> goalPoseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier, Consumer<ChassisSpeeds> targetFieldSpeedsConsumer,
            Config config)
    {
        this.currentPoseSupplier = currentPoseSupplier;
        this.goalPoseSupplier = goalPoseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
        this.targetFieldSpeedsConsumer = targetFieldSpeedsConsumer;

        this.config = config;
        this.limiter = config.standardLimiter;

        drivePosTol = new TunableNumber("Align/"+name+"/drivePosTolInches", config.drivePosTol.in(Inches));
        driveVelTol = new TunableNumber("Align/"+name+"/driveVelTolInches", config.driveVelTol.in(InchesPerSecond));
        thetaPosTol = new TunableNumber("Align/"+name+"/thetaPosTolDegrees", config.thetaPosTol.in(Degrees));
        thetaVelTol = new TunableNumber("Align/"+name+"/thetaVelTolDegrees", config.thetaVelTol.in(DegreesPerSecond));
        finalAlignDist = new TunableNumber("Align/"+name+"/finalAlignDistFeet", config.finalAlignDist.in(Feet));

        goalPosePub = NetworkTableInstance.getDefault().getStructTopic("Align/"+name+"/Goal Pose", Pose2d.struct).publish();
        targetPosePub = NetworkTableInstance.getDefault().getStructTopic("Align/"+name+"/Target Pose", Pose2d.struct).publish();
    }

    @Override
    public void initialize() {
        /*
         * To initialize our profiled controllers, we want to reset their setpoints to the current robot position and speeds.
         */

        Pose2d currentPose = currentPoseSupplier.get();
        Pose2d goalPose = goalPoseSupplier.get();
        Pose2d goalRelPose = currentPose.relativeTo(goalPose);
        if (goalRelPose.getTranslation().getNorm() > finalAlignDist.get()) {
            goalPose = goalPose.plus(new Transform2d(finalAlignDist.get(), 0, Rotation2d.kZero));
            goalRelPose = currentPose.relativeTo(goalPose);
        }

        ChassisSpeeds currentSpeeds = fieldSpeedsSupplier.get();
        Translation2d linearVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        Translation2d goalRelVel = linearVelocity.rotateBy(goalRelPose.getRotation().unaryMinus());

        xController.reset(
            goalRelPose.getX(),
            goalRelVel.getX()
        );

        yController.reset(
            goalRelPose.getY(),
            goalRelVel.getY()
        );

        thetaController.reset(
            currentPose.getRotation().getRadians(),
            currentSpeeds.omegaRadiansPerSecond
        );

        lastSetpointPose = currentPose;
        lastSetpointSpeeds = currentSpeeds;
    }

    @Override
    public void execute() {
        changeTunable();

        Pose2d currentPose = currentPoseSupplier.get();
        Pose2d goalPose = goalPoseSupplier.get();
        goalPosePub.set(goalPose);
        Pose2d goalRelPose = currentPose.relativeTo(goalPose);
        double finalDist = finalAlignDist.get();
        boolean isFinalAlignment = goalRelPose.getTranslation().getNorm() <= finalDist;
        if (!isFinalAlignment) {
            double finalAlignXOffset = config.alignBackwards ? finalDist : -finalDist;
            goalPose = goalPose.plus(new Transform2d(finalAlignXOffset, 0, Rotation2d.kZero));
            goalRelPose = currentPose.relativeTo(goalPose);
        }
        double distToGoal = goalRelPose.getTranslation().getNorm();        

        /*
         * We reset the controllers with the last setpoint, which is stored field-relative.
         * This is usually redundant, but is useful if our goal pose changes.
         */
        Pose2d goalRelLastSetpointPose = lastSetpointPose.relativeTo(goalPose);
        ChassisSpeeds goalRelLastSetpointSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(lastSetpointSpeeds, goalPose.getRotation());

        xController.reset(
            goalRelLastSetpointPose.getX(),
            goalRelLastSetpointSpeeds.vxMetersPerSecond
        );
        yController.reset(
            goalRelLastSetpointPose.getY(),
            goalRelLastSetpointSpeeds.vyMetersPerSecond
        );
        thetaController.reset(
            goalRelLastSetpointPose.getRotation().getRadians(),
            lastSetpointSpeeds.omegaRadiansPerSecond
        );

        /*
         * We update the profile constraints based on our current limiter values.
         * Because we have chosen to use independent x and y controllers for control over
         * the path's shape, it is important to do this with their combined linear result in mind.
         */
        updateConstraints(distToGoal);
        Rotation2d goalRelTrlAngle = goalRelPose.getTranslation().getAngle(); // TODO: this needs to not shrink y as it approaches?

        double xScale = goalRelTrlAngle.getCos();
        xController.setConstraints(new TrapezoidProfile.Constraints(
            limiter.linearTopSpeed.in(MetersPerSecond) * xScale,
            limiter.linearAcceleration.in(MetersPerSecondPerSecond)
        ));

        double yScale = goalRelTrlAngle.getSin();
        yController.setConstraints(new TrapezoidProfile.Constraints(
            limiter.linearTopSpeed.in(MetersPerSecond) * yScale,
            limiter.linearAcceleration.in(MetersPerSecondPerSecond)
        ));

        thetaController.setConstraints(new TrapezoidProfile.Constraints(
            limiter.angularTopSpeed.in(RadiansPerSecond),
            limiter.angularAcceleration.in(RadiansPerSecondPerSecond)
        ));

        /*
         * Calculate the next profile setpoint.
         * We use a threshold that slows down the robot for the final bit of alignment.
         */
        double finalAlignSpeed = Math.min(limiter.linearTopSpeed.in(MetersPerSecond), config.finishLimiter.linearTopSpeed.in(MetersPerSecond));
        double finalAlignXSpeed = config.alignBackwards ? -finalAlignSpeed : finalAlignSpeed;
        xController.calculate(
            goalRelPose.getX(),
            new TrapezoidProfile.State(0, isFinalAlignment ? 0 : finalAlignXSpeed)
        );

        yController.calculate(
            goalRelPose.getY()
        );

        thetaController.calculate(
            goalRelPose.getRotation().getRadians()
        );

        lastSetpointPose = new Pose2d(
            xController.getSetpoint().position,
            yController.getSetpoint().position,
            new Rotation2d(thetaController.getSetpoint().position)
        ).plus(goalPose.minus(Pose2d.kZero));
        targetPosePub.set(lastSetpointPose);

        lastSetpointSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xController.getSetpoint().velocity,
            yController.getSetpoint().velocity,
            thetaController.getSetpoint().velocity,
            goalPose.getRotation()
        );

        targetFieldSpeedsConsumer.accept(lastSetpointSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
    }

    private void updateConstraints(double distToGoal) {
        //--- Determine limiter values to be used in profile generation
        limiter.copyFrom(config.standardLimiter);

        // if within final alignment distance, use final alignment limiting
        double finalDist = finalAlignDist.get();
        if (distToGoal <= finalDist) {
            limiter.copyFrom(config.finishLimiter);
        }

        // ensure not faster than reference (e.g. reference limited based on elevator height)
        if (config.referenceLimiter != null) {
            limiter.setToSlowestOf(limiter, config.referenceLimiter);
        }
    }

    private void changeTunable() {
        pathDriveKP.poll();
        pathDriveKD.poll();
        pathTurnKP.poll();
        pathTurnKD.poll();
        drivePosTol.poll();
        driveVelTol.poll();
        thetaPosTol.poll();
        thetaVelTol.poll();
        finalAlignDist.poll();

        int hash = hashCode();
        if (pathDriveKP.hasChanged(hash) || pathDriveKD.hasChanged(hash)) {
            xController.setP(pathDriveKP.get());
            xController.setD(pathDriveKD.get());
            yController.setP(pathDriveKP.get());
            yController.setD(pathDriveKD.get());
        }
        if (pathTurnKP.hasChanged(hash) || pathTurnKD.hasChanged(hash)) {
            thetaController.setP(pathTurnKP.get());
            thetaController.setD(pathTurnKD.get());
        }
        if (drivePosTol.hasChanged(hash) || driveVelTol.hasChanged(hash)) {
            xController.setTolerance(drivePosTol.get(), driveVelTol.get());
            yController.setTolerance(drivePosTol.get(), driveVelTol.get());
        }
        if (thetaPosTol.hasChanged(hash) || thetaVelTol.hasChanged(hash)) {
            thetaController.setTolerance(thetaPosTol.get(), thetaVelTol.get());
        }
    }
}
