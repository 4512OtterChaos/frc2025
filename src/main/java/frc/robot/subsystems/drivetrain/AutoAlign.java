package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TunableNumber;

public class AutoAlign extends Command {
    /** Limiter for normal alignment speeds */
    public SwerveDriveLimiter standardLimiter = kAlignLimiter.copy();

    /** Limiter for final alignment speeds to goal pose */
    public SwerveDriveLimiter finishLimiter = kAlignFinalLimiter.copy();

    /** Reference swerve limiter. The auto-align limiting will never be more aggressive than this. */
    public SwerveDriveLimiter referenceLimiter;

    private final ProfiledPIDController driveController = new ProfiledPIDController(
        kPathDriveKP, kPathDriveKI, kPathDriveKD,
        new TrapezoidProfile.Constraints(0, 0)
    );
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        kPathTurnKP, kPathTurnKI, kPathTurnKD,
        new TrapezoidProfile.Constraints(0, 0)
    );

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
        public Distance drivePosTol = Meters.of(kAlignDrivePosTol);
        public LinearVelocity driveVelTol = MetersPerSecond.of(kAlignDriveVelTol);
        public Angle thetaPosTol = Radians.of(kAlignTurnPosTol);
        public AngularVelocity thetaVelTol = RadiansPerSecond.of(kAlignTurnVelTol);
        public Distance finalAlignDist = Meters.of(kFinalAlignDist);
        public boolean alignBackwards = false;
    }

    // Instance tunables
    public final Config config;
    private final TunableNumber drivePosTol;
    private final TunableNumber driveVelTol;
    private final TunableNumber thetaPosTol;
    private final TunableNumber thetaVelTol;
    private final TunableNumber finalAlignDist;

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
        drivePosTol = new TunableNumber("Align/"+name+"/drivePosTolInches", config.drivePosTol.in(Inches));
        driveVelTol = new TunableNumber("Align/"+name+"/driveVelTolInches", config.driveVelTol.in(InchesPerSecond));
        thetaPosTol = new TunableNumber("Align/"+name+"/thetaPosTolDegrees", config.thetaPosTol.in(Degrees));
        thetaVelTol = new TunableNumber("Align/"+name+"/thetaVelTolDegrees", config.thetaVelTol.in(DegreesPerSecond));
        finalAlignDist = new TunableNumber("Align/"+name+"/finalAlignDistFeet", config.finalAlignDist.in(Feet));
    }

    @Override
    public void initialize() {
        /*
         * To initialize our profiled controllers, we want to reset their setpoints to the current robot position and speeds.
         */

        Pose2d currentPose = currentPoseSupplier.get();
        // Translation error to goal
        Translation2d trlError = currentPose.getTranslation().minus(goalPoseSupplier.get().getTranslation());

        ChassisSpeeds currentSpeeds = fieldSpeedsSupplier.get();
        Translation2d linearVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        driveController.reset(
            // our current "position" is our distance to the goal, with our goal as 0
            trlError.getNorm(),
            // thus, our current "velocity" is the magnitude of the field velocity towards the goal, negated.
            -linearVelocity.rotateBy(trlError.getAngle().unaryMinus()).getX()
        );

        thetaController.reset(
            currentPose.getRotation().getRadians(),
            currentSpeeds.omegaRadiansPerSecond
        );
    }

    @Override
    public void execute() {
        // Update constraints
        updateConstraints();
    }

    @Override
    public void end(boolean interrupted) {
    }

    private void updateConstraints() {
        drivePosTol.poll();
        driveVelTol.poll();
        thetaPosTol.poll();
        thetaVelTol.poll();
        finalAlignDist.poll();

        int hash = hashCode();
        if (drivePosTol.hasChanged(hash) || driveVelTol.hasChanged(hash)) {
            driveController.setTolerance(drivePosTol.get(), driveVelTol.get());
        }
        if (thetaPosTol.hasChanged(hash) || thetaVelTol.hasChanged(hash)) {
            thetaController.setTolerance(thetaPosTol.get(), thetaVelTol.get());
        }

        driveController.setConstraints(new TrapezoidProfile.Constraints(0, 0));
        thetaController.setConstraints(new TrapezoidProfile.Constraints(0, 0));
    }
}
