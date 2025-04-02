package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ProfiledPIDController;
import frc.robot.util.TrapezoidProfile;
import frc.robot.util.TunableNumber;

public class AutoAlign extends Command {

    private final CommandSwerveDrivetrain swerve;

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

    private final SwerveRequest.RobotCentric applyPathRobotSpeeds = new SwerveRequest.RobotCentric();

    private Pose2d lastSetpointPose;
    private ChassisSpeeds lastSetpointSpeeds;

    private final Supplier<Pose2d> goalPoseSupplier;

    public static class Config {
        /** Limiter for normal alignment speeds */
        public SwerveDriveLimiter standardLimiter = kAlignLimiter.copy();
        /** Limiter for final alignment speeds to goal pose */
        public SwerveDriveLimiter finalLimiter = kAlignFinalLimiter.copy();
        /** Reference swerve limiter. The auto-align speeds will never be more aggressive than this. */
        public SwerveDriveLimiter referenceLimiter;

        public Distance drivePosTol = Meters.of(kAlignDrivePosTol);
        public LinearVelocity driveVelTol = MetersPerSecond.of(kAlignDriveVelTol);
        public Angle thetaPosTol = Radians.of(kAlignTurnPosTol);
        public AngularVelocity thetaVelTol = RadiansPerSecond.of(kAlignTurnVelTol);

        public Distance finalAlignDist = Meters.of(kFinalAlignDist);

        public LinearVelocity driveDeadband = InchesPerSecond.of(0.75);
        public AngularVelocity turnDeadband = DegreesPerSecond.of(3);

        public boolean alignBackwards = false;

        public boolean runForever = false;
    }

    // Instance tunables
    private final Config config;

    private final TunableNumber pathDriveKP;
    private final TunableNumber pathDriveKD;
    private final TunableNumber pathTurnKP;
    private final TunableNumber pathTurnKD;
    private final TunableNumber drivePosTol;
    private final TunableNumber driveVelTol;
    private final TunableNumber thetaPosTol;
    private final TunableNumber thetaVelTol;

    private final TunableNumber driveSpeedNormal;
    private final TunableNumber driveAccelNormal;
    private final TunableNumber turnSpeedNormal;
    private final TunableNumber turnAccelNormal;

    private final TunableNumber finalAlignDist;

    private final TunableNumber driveSpeedFinal;
    private final TunableNumber driveAccelFinal;
    private final TunableNumber turnSpeedFinal;
    private final TunableNumber turnAccelFinal;

    private final TunableNumber driveDeadbandInches;
    private final TunableNumber turnDeadband;

    private final StructPublisher<Pose2d> goalPosePub;
    private final StructPublisher<Pose2d> setpointPosePub;

    private final DoublePublisher errorDistPub;
    private final DoublePublisher errorXPub;
    private final DoublePublisher errorYPub;
    private final DoublePublisher errorRotPub;

    public AutoAlign(
            String name,
            CommandSwerveDrivetrain swerve,
            Supplier<Pose2d> goalPoseSupplier)
    {
        this(name, swerve, goalPoseSupplier, new Config());
    }

    public AutoAlign(
            String name,
            CommandSwerveDrivetrain swerve,
            Supplier<Pose2d> goalPoseSupplier,
            Config config)
    {
        this.swerve = swerve;
        this.goalPoseSupplier = goalPoseSupplier;

        this.config = config;
        this.limiter = config.standardLimiter.copy();

        pathDriveKP = new TunableNumber("Align/"+name+"/Controller/pathDriveKP", kPathDriveKP);
        pathDriveKD = new TunableNumber("Align/"+name+"/Controller/pathDriveKD", kPathDriveKD);
        pathTurnKP = new TunableNumber("Align/"+name+"/Controller/pathTurnKP", kPathTurnKP);
        pathTurnKD = new TunableNumber("Align/"+name+"/Controller/pathTurnKD", kPathTurnKD);
        drivePosTol = new TunableNumber("Align/"+name+"/Controller/drivePosTolInches", config.drivePosTol.in(Inches));
        driveVelTol = new TunableNumber("Align/"+name+"/Controller/driveVelTolInches", config.driveVelTol.in(InchesPerSecond));
        thetaPosTol = new TunableNumber("Align/"+name+"/Controller/thetaPosTolDegrees", config.thetaPosTol.in(Degrees));
        thetaVelTol = new TunableNumber("Align/"+name+"/Controller/thetaVelTolDegrees", config.thetaVelTol.in(DegreesPerSecond));

        driveSpeedNormal = new TunableNumber("Align/"+name+"/Standard Limiter/driveSpeedNormal", config.standardLimiter.linearTopSpeed.in(MetersPerSecond));
        driveAccelNormal = new TunableNumber("Align/"+name+"/Standard Limiter/driveAccelNormal", config.standardLimiter.linearAcceleration.in(MetersPerSecondPerSecond));
        turnSpeedNormal = new TunableNumber("Align/"+name+"/Standard Limiter/turnSpeedNormal", config.standardLimiter.angularTopSpeed.in(RadiansPerSecond));
        turnAccelNormal = new TunableNumber("Align/"+name+"/Standard Limiter/turnAccelNormal", config.standardLimiter.angularAcceleration.in(RadiansPerSecondPerSecond));

        finalAlignDist = new TunableNumber("Align/"+name+"/finalAlignDist", config.finalAlignDist.in(Meters));

        driveSpeedFinal = new TunableNumber("Align/"+name+"/Final Limiter/driveSpeedFinal", config.finalLimiter.linearTopSpeed.in(MetersPerSecond));
        driveAccelFinal = new TunableNumber("Align/"+name+"/Final Limiter/driveAccelFinal", config.finalLimiter.linearAcceleration.in(MetersPerSecondPerSecond));
        turnSpeedFinal = new TunableNumber("Align/"+name+"/Final Limiter/turnSpeedFinal", config.finalLimiter.angularTopSpeed.in(RadiansPerSecond));
        turnAccelFinal = new TunableNumber("Align/"+name+"/Final Limiter/turnAccelFinal", config.finalLimiter.angularAcceleration.in(RadiansPerSecondPerSecond));

        driveDeadbandInches = new TunableNumber("Align/"+name+"/driveDeadbandInches", config.driveDeadband.in(InchesPerSecond));
        turnDeadband = new TunableNumber("Align/"+name+"/turnDeadbandDegrees", config.turnDeadband.in(DegreesPerSecond));

        goalPosePub = NetworkTableInstance.getDefault().getStructTopic("Align/"+name+"/Goal Pose", Pose2d.struct).publish();
        setpointPosePub = NetworkTableInstance.getDefault().getStructTopic("Align/"+name+"/Setpoint Pose", Pose2d.struct).publish();

        errorDistPub = NetworkTableInstance.getDefault().getDoubleTopic("Align/"+name+"/Dist Error Inches").publish();
        errorXPub = NetworkTableInstance.getDefault().getDoubleTopic("Align/"+name+"/X Error Inches").publish();
        errorYPub = NetworkTableInstance.getDefault().getDoubleTopic("Align/"+name+"/Y Error Inches").publish();
        errorRotPub = NetworkTableInstance.getDefault().getDoubleTopic("Align/"+name+"/Rot Error Degrees").publish();

        addRequirements(swerve);
        setName("AutoAlign");
    }

    @Override
    public void initialize() {
        /*
         * To initialize our profiled controllers, we want to reset their setpoints to the current robot position and speeds.
         */

        Pose2d currentPose = swerve.getGlobalPoseEstimate();
        Pose2d goalPose = goalPoseSupplier.get();
        swerve.alignGoal = goalPose;
        Pose2d goalRelPose = currentPose.relativeTo(goalPose);

        ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getState().Speeds, currentPose.getRotation());
        ChassisSpeeds goalRelVel = ChassisSpeeds.fromFieldRelativeSpeeds(currentSpeeds, goalPose.getRotation());

        xController.reset(
            goalRelPose.getX(),
            goalRelVel.vxMetersPerSecond
        );

        yController.reset(
            goalRelPose.getY(),
            goalRelVel.vyMetersPerSecond
        );

        thetaController.reset(
            goalRelPose.getRotation().getRadians(),
            goalRelVel.omegaRadiansPerSecond
        );

        lastSetpointPose = currentPose;
        lastSetpointSpeeds = currentSpeeds;
    }

    @Override
    public void execute() {
        changeTunable();

        swerve.aligning = true;
        Pose2d currentPose = swerve.getGlobalPoseEstimate();
        Pose2d goalPose = goalPoseSupplier.get();
        swerve.alignGoal = goalPose;
        goalPosePub.set(goalPose);
        Pose2d goalRelPose = currentPose.relativeTo(goalPose);

        swerve.atGoal = MathUtil.isNear(0, goalRelPose.getX(), Units.inchesToMeters(drivePosTol.get()));
        swerve.atGoal &= MathUtil.isNear(0, goalRelPose.getY(), Units.inchesToMeters(drivePosTol.get()));
        swerve.atGoal &= MathUtil.isNear(0, goalRelPose.getRotation().getRadians(), Units.degreesToRadians(thetaPosTol.get()), -Math.PI, Math.PI);

        double finalDist = finalAlignDist.get();
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

        ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getState().Speeds, currentPose.getRotation());
        ChassisSpeeds goalRelVel = ChassisSpeeds.fromFieldRelativeSpeeds(currentSpeeds, goalPose.getRotation());
        ChassisSpeeds speedsError = goalRelVel.minus(currentSpeeds);
        swerve.atSetpointVel = MathUtil.isNear(0, speedsError.vxMetersPerSecond, Units.inchesToMeters(driveVelTol.get()));
        swerve.atSetpointVel &= MathUtil.isNear(0, speedsError.vyMetersPerSecond, Units.inchesToMeters(driveVelTol.get()));
        swerve.atSetpointVel &= MathUtil.isNear(0, speedsError.omegaRadiansPerSecond, Units.inchesToMeters(thetaVelTol.get()));

        /*
         * We update the profile constraints based on our current limiter values.
         * Because we have chosen to use independent x and y controllers for control over
         * the path's shape, it is important to do this with their combined linear result in mind.
         */
        updateConstraints(distToGoal);

        xController.setConstraints(new TrapezoidProfile.Constraints(
            limiter.linearTopSpeed.in(MetersPerSecond),
            limiter.linearAcceleration.in(MetersPerSecondPerSecond)
        ));

        yController.setConstraints(new TrapezoidProfile.Constraints(
            limiter.linearTopSpeed.in(MetersPerSecond),
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
        boolean isFinalAlignment = distToGoal <= finalDist;
        double finalAlignXOffset = config.alignBackwards ? finalDist : -finalDist;
        double finalAlignSpeed = Math.min(limiter.linearTopSpeed.in(MetersPerSecond), config.finalLimiter.linearTopSpeed.in(MetersPerSecond));
        double finalAlignXSpeed = config.alignBackwards ? -finalAlignSpeed : finalAlignSpeed;
        ChassisSpeeds pidSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xController.calculate(
                goalRelPose.getX(),
                new TrapezoidProfile.State(
                    isFinalAlignment ? 0 : finalAlignXOffset,
                    isFinalAlignment ? 0 : finalAlignXSpeed
                )
            ),

            yController.calculate(
                goalRelPose.getY()
            ),

            thetaController.calculate(
                goalRelPose.getRotation().getRadians()
            ),

            goalPose.getRotation()
        );

        errorXPub.set(xController.getPositionError());
        errorYPub.set(yController.getPositionError());
        errorDistPub.set(Math.hypot(xController.getPositionError(), yController.getPositionError()));
        errorRotPub.set(Units.radiansToDegrees(thetaController.getPositionError()));

        lastSetpointPose = goalPose.plus(new Transform2d(
            xController.getSetpoint().position,
            yController.getSetpoint().position,
            new Rotation2d(thetaController.getSetpoint().position)
        ));
        setpointPosePub.set(lastSetpointPose);

        lastSetpointSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xController.getSetpoint().velocity,
            yController.getSetpoint().velocity,
            thetaController.getSetpoint().velocity,
            goalPose.getRotation()
        );

        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(lastSetpointSpeeds.plus(pidSpeeds), currentPose.getRotation());
        swerve.setControl(applyPathRobotSpeeds
            .withVelocityX(robotSpeeds.vxMetersPerSecond)
            .withVelocityY(robotSpeeds.vyMetersPerSecond)
            .withRotationalRate(robotSpeeds.omegaRadiansPerSecond)
            .withDriveRequestType(DriveRequestType.Velocity)
        );
    }

    @Override
    public boolean isFinished() {
        swerve.aligned = swerve.aligning && swerve.atSetpointVel && swerve.atGoal;
        return swerve.aligned && !config.runForever;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.aligning = false;
    }

    private void updateConstraints(double distToGoal) {
        //--- Determine limiter values to be used in profile generation
        limiter.copyFrom(config.standardLimiter);

        // if within final alignment distance, use final alignment limiting
        double finalDist = finalAlignDist.get();
        if (distToGoal <= finalDist) {
            limiter.copyFrom(config.finalLimiter);
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

        driveSpeedNormal.poll();
        driveAccelNormal.poll();
        turnSpeedNormal.poll();
        turnAccelNormal.poll();

        finalAlignDist.poll();

        driveSpeedFinal.poll();
        driveAccelFinal.poll();
        turnSpeedFinal.poll();
        turnAccelFinal.poll();

        driveDeadbandInches.poll();
        turnDeadband.poll();

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

        if (driveSpeedNormal.hasChanged(hash) || driveAccelNormal.hasChanged(hash)
                || turnSpeedNormal.hasChanged(hash) || turnAccelNormal.hasChanged(hash)) {
            config.standardLimiter.linearTopSpeed = MetersPerSecond.of(driveSpeedNormal.get());
            config.standardLimiter.linearAcceleration = MetersPerSecondPerSecond.of(driveAccelNormal.get());
            config.standardLimiter.linearDeceleration = MetersPerSecondPerSecond.of(driveAccelNormal.get());
            config.standardLimiter.angularTopSpeed = RadiansPerSecond.of(turnSpeedNormal.get());
            config.standardLimiter.angularAcceleration = RadiansPerSecondPerSecond.of(turnAccelNormal.get());
            config.standardLimiter.angularDeceleration = RadiansPerSecondPerSecond.of(turnAccelNormal.get());
        }

        if (driveSpeedFinal.hasChanged(hash) || driveAccelFinal.hasChanged(hash)
                || turnSpeedFinal.hasChanged(hash) || turnAccelFinal.hasChanged(hash)) {
            config.finalLimiter.linearTopSpeed = MetersPerSecond.of(driveSpeedFinal.get());
            config.finalLimiter.linearAcceleration = MetersPerSecondPerSecond.of(driveAccelFinal.get());
            config.finalLimiter.linearDeceleration = MetersPerSecondPerSecond.of(driveAccelFinal.get());
            config.finalLimiter.angularTopSpeed = RadiansPerSecond.of(turnSpeedFinal.get());
            config.finalLimiter.angularAcceleration = RadiansPerSecondPerSecond.of(turnAccelFinal.get());
            config.finalLimiter.angularDeceleration = RadiansPerSecondPerSecond.of(turnAccelFinal.get());
        }

        if (driveDeadbandInches.hasChanged(hash) || turnDeadband.hasChanged(hash)) {
            applyPathRobotSpeeds.Deadband = Units.inchesToMeters(driveDeadbandInches.get());
            applyPathRobotSpeeds.RotationalDeadband = Units.degreesToRadians(turnDeadband.get());
        }
    }
}
