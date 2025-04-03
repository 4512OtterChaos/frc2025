package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.FieldUtil.*;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.FieldUtil.AlgaeHeight;
import frc.robot.util.FieldUtil.Alignment;
import frc.robot.util.FieldUtil.CoralStation;
import frc.robot.util.FieldUtil.ReefFace;
import frc.robot.util.FieldUtil;
import frc.robot.util.TunableNumber;


public class Superstructure {
    private CommandSwerveDrivetrain swerve;
    private Manipulator manipulator;
    private Funnel funnel;
    private Elevator elevator;

    private SuperstructureViz viz = new SuperstructureViz();

    public Superstructure(CommandSwerveDrivetrain swerve, Manipulator manipulator, Funnel funnel, Elevator elevator) {
        this.swerve = swerve;
        this.manipulator = manipulator;
        this.funnel = funnel;
        this.elevator = elevator;
    }

    private final TunableNumber driveSpeedNormal = new TunableNumber("Driver/driveSpeedNormal", kDriveSpeed);
    private final TunableNumber driveAccelNormal = new TunableNumber("Driver/driveAccelNormal", kLinearAccel);
    private final TunableNumber driveDecelNormal = new TunableNumber("Driver/driveDecelNormal", kLinearDecel);
    private final TunableNumber turnSpeedNormal = new TunableNumber("Driver/turnSpeedNormal", kTurnSpeed);
    private final TunableNumber turnAccelNormal = new TunableNumber("Driver/turnAccelNormal", kAngularAccel);
    private final TunableNumber turnDecelNormal = new TunableNumber("Driver/turnDecelNormal", kAngularDecel);

    private final TunableNumber driveSpeedTippy = new TunableNumber("Driver/driveSpeedTippy", kDriveSpeedTippy);
    private final TunableNumber driveAccelTippy = new TunableNumber("Driver/driveAccelTippy", kLinearAccelTippy);
    private final TunableNumber driveDecelTippy = new TunableNumber("Driver/driveDecelTippy", kLinearDecelTippy);
    private final TunableNumber turnSpeedTippy = new TunableNumber("Driver/turnSpeedTippy", kTurnSpeedTippy);
    private final TunableNumber turnAccelTippy = new TunableNumber("Driver/turnAccelTippy", kAngularAccelTippy);
    private final TunableNumber turnDecelTippy = new TunableNumber("Driver/turnDecelTippy", kAngularDecelTippy);
    
    private final TunableNumber reefCoralXOffset = new TunableNumber("Align/reefCoralXOffset", Units.inchesToMeters(0.75));
    private final TunableNumber reefAlgaeXOffset = new TunableNumber("Align/reefAlgaeXOffset", Units.inchesToMeters(1));

    private final TunableNumber netAlgaeReleaseHeight = new TunableNumber("Commands/algaeShotElevHeightOffsetInches", 12);

    public void periodic() {
        changeTunable();

        adjustDriving();

        viz.update(elevator.getHeight(), manipulator.getPosition(), funnel.getPosition());
    }

    //########## Alignnment Commands

    public Pose2d adjustAlignPoseSlow(Pose2d goalPose, Pose2d currentPose, double slowDistMeters) {
        var currRelToGoal = currentPose.relativeTo(goalPose);
        double angularErrorRots = Math.abs(currRelToGoal.getRotation().getRotations());
        double trlErrorMeters = currRelToGoal.getTranslation().getNorm();

        double trlAngleToGoalError = Math.abs(currRelToGoal.getTranslation().getAngle().plus(Rotation2d.kPi).getRotations());

        double xOffsetAngular = angularErrorRots * 2;
        double xOffsetTrlY = Math.min(trlAngleToGoalError * trlErrorMeters * 7, 1);

        double xOffset = Math.max(xOffsetAngular, xOffsetTrlY);
        if (trlAngleToGoalError > 0.25) {
            trlAngleToGoalError = 0.5 - trlAngleToGoalError;
            xOffsetTrlY = -Math.min(trlAngleToGoalError * trlErrorMeters * 7, 1);
            xOffsetAngular = -xOffsetAngular;
            xOffset = Math.min(xOffsetAngular, xOffsetTrlY);
        }
        var adjGoal = goalPose.plus(new Transform2d(-xOffset, 0, Rotation2d.kZero));
        if (trlErrorMeters < slowDistMeters) {
            var lerpPose = currentPose.interpolate(adjGoal, 0.2 / adjGoal.getTranslation().getDistance(currentPose.getTranslation())); // idk
            adjGoal = new Pose2d(lerpPose.getTranslation(), adjGoal.getRotation());
        }
        return adjGoal;
    }

    // public Pose2d adjustAlignPose(Pose2d goalPose, Pose2d currentPose) {
    //     var currRelToGoal = currentPose.relativeTo(goalPose);
    //     double angularErrorRots = Math.abs(currRelToGoal.getRotation().getRotations());
    //     double trlErrorMeters = currRelToGoal.getTranslation().getNorm();

    //     if ((trlErrorMeters < kAlignAdjustDrivePosTol) && (angularErrorRots < kAlignAdjustTurnPosTol)){
    //         return goalPose;
    //     }

    //     double trlAngleToGoalError = Math.abs(currRelToGoal.getTranslation().getAngle().plus(Rotation2d.kPi).getRotations());

    //     double xOffsetAngular = angularErrorRots * 2;
    //     double xOffsetTrlY = Math.min(trlAngleToGoalError * trlErrorMeters * 7, 1);

    //     double xOffset = Math.max(xOffsetAngular, xOffsetTrlY);
    //     return goalPose.plus(new Transform2d(-xOffset, 0, Rotation2d.kZero));
    // }

    //########## Sequencing commands

    public Command algaeShoot(){
        return sequence(
            either(none(), elevator.setAlgaeL2C(), ()-> elevator.getHeight().lt(ElevatorConstants.kAlgaeL3Height)),
            parallel(
                elevator.setL4C(),
                sequence(
                    waitUntil(()->elevator.getHeight().in(Meters) >= ElevatorConstants.kL4Height.minus(Inches.of(netAlgaeReleaseHeight.get())).in(Meters))
                        .deadlineFor(
                            manipulator.scoreCoralC().asProxy()
                    ),
                    manipulator.scoreAlgaeC().withTimeout(0.75).asProxy()
                ),
                sequence(
                    waitSeconds(0.25),
                    swerve.drive(()->new ChassisSpeeds(1, 0, 0)).withTimeout(0.75).andThen(swerve.stop())
                )
            )
        ).withName("AlgaeShoot");
    }

    public Command feedCoralSequenceC() {
        return manipulator.feedCoralSequenceC();
    }

    public Command feedCoralFastSequenceC() {
        return manipulator.feedCoralFastSequenceC();
    }

    //########## Auto commands

    public Command autoScore(Alignment alignment, ElevatorHeight scorePos) {
        return autoScore(() -> ReefFace.getClosest(swerve.getGlobalPoseEstimate(), alignment).getAlignmentPose(alignment), scorePos);
    }

    public Command autoScore(ReefFace face, Alignment alignment, ElevatorHeight scorePos) {
        return autoScore(() -> face.getAlignmentPose(alignment), scorePos);
    }

    public Command autoScore(Supplier<Pose2d> goalSupplier, ElevatorHeight scorePos) {
        Command elevatorCommand;
        switch (scorePos) {
            case L1 -> elevatorCommand = elevator.setL1C();
            case L2 -> elevatorCommand = elevator.setL2C();
            case L3 -> elevatorCommand = elevator.setL3C();
            case L4 -> elevatorCommand = elevator.setL4C();
            default -> elevatorCommand = elevator.setL4C();
        }

        return sequence(
            parallel(
                swerve.alignToReef(goalSupplier, false),
                sequence(
                    waitUntil(swerve.isFinalAlignment),
                    elevatorCommand
                )
            ),
            manipulator.scoreCoralC().asProxy().withTimeout(0.4)
        ).withName("AlignToReefAndScore" + scorePos.toString());
    }

    public Command autoCoralStation(CoralStation coralStation, Alignment alignment){
        Trigger simSkipCoral = new Trigger(() -> {
            Pose2d swervePose = swerve.getGlobalPoseEstimate();
            Pose2d goalPose = swerve.getAlignGoal();
            double dist = goalPose.getTranslation().getDistance(swervePose.getTranslation());
            return dist < 0.1 && Robot.isSimulation();
        }).debounce(0.5).and(swerve.isAligning);

        return swerve.alignToStation(() -> FieldUtil.offsetCoralStation(coralStation.getPose(), alignment), false)
            .until(manipulator.isCoralDetected().or(simSkipCoral));
    }

    public Command autoAlgaePickUp() {
        return autoAlgaePickUp(() -> ReefFace.getClosest(swerve.getGlobalPoseEstimate(), Alignment.CENTER));
    }

    public Command autoAlgaePickUp(Supplier<ReefFace> face) {
        return autoAlgaePickUp(()-> face.get().getAlignmentPose(Alignment.CENTER), ()-> face.get().algaeHeight);
    }

    public Command autoAlgaePickUp(ReefFace face) {
        return autoAlgaePickUp(() -> face.getAlignmentPose(Alignment.CENTER), () -> face.algaeHeight);
    }

    public Command autoAlgaePickUp(Supplier<Pose2d> goalSupplier, Supplier<AlgaeHeight> algaeHeight) {
        Trigger simSkipAlgae = new Trigger(() -> {
            Pose2d swervePose = swerve.getGlobalPoseEstimate();
            Pose2d goalPose = swerve.getAlignGoal();
            double dist = goalPose.getTranslation().getDistance(swervePose.getTranslation());
            return dist < 0.1 && Robot.isSimulation();
        }).debounce(0.5).and(swerve.isAligning);

        return sequence(
            parallel(
                sequence(
                    waitUntil(swerve.isFinalAlignment),
                    parallel(
                        manipulator.scoreCoralC().asProxy().until(manipulator.hasAlgae.or(simSkipAlgae))
                    )
                ).deadlineFor(
                    swerve.alignToReef(goalSupplier, true),
                    elevator.setHeightC(()-> elevator.getAlgaeHeight(algaeHeight.get())).repeatedly()
                )
            )
        ).withName("AlignToReefAndPickUpAlgae" + algaeHeight.toString());
    }

    public void adjustDriving() {
        double elevatorPercentTravel = elevator.getHeight().div(ElevatorConstants.kMaxTravel).magnitude();

        swerve.driveSpeed = MetersPerSecond.of(elevatorPercentTravel*(driveSpeedTippy.get() - driveSpeedNormal.get()) + driveSpeedNormal.get());
        swerve.turnSpeed = RadiansPerSecond.of(elevatorPercentTravel*(turnSpeedTippy.get() - turnSpeedNormal.get()) + turnSpeedNormal.get());

        swerve.limiter.setToLerpOf(kStandardLimiter, kTippyLimiter, elevatorPercentTravel);
    }

    private void changeTunable() {
        driveSpeedNormal.poll();
        driveAccelNormal.poll();
        driveDecelNormal.poll();
        turnSpeedNormal.poll();
        turnAccelNormal.poll();
        turnDecelNormal.poll();

        driveSpeedTippy.poll();
        driveAccelTippy.poll();
        driveDecelTippy.poll();
        turnSpeedTippy.poll();
        turnAccelTippy.poll();
        turnDecelTippy.poll();

        reefCoralXOffset.poll();
        reefAlgaeXOffset.poll();

        netAlgaeReleaseHeight.poll();
        
        int hash = hashCode();
        if (reefCoralXOffset.hasChanged(hash) || reefAlgaeXOffset.hasChanged(hash)) {
            FieldUtil.ReefFace.updatePoses(Meters.of(reefCoralXOffset.get()), Meters.of(reefAlgaeXOffset.get()));
        }

        if (driveSpeedNormal.hasChanged(hash) || driveAccelNormal.hasChanged(hash) || driveDecelNormal.hasChanged(hash)
                || turnSpeedNormal.hasChanged(hash) || turnAccelNormal.hasChanged(hash) || turnDecelNormal.hasChanged(hash)) {
            kStandardLimiter.linearTopSpeed = MetersPerSecond.of(driveSpeedNormal.get());
            kStandardLimiter.linearAcceleration = MetersPerSecondPerSecond.of(driveAccelNormal.get());
            kStandardLimiter.linearDeceleration = MetersPerSecondPerSecond.of(driveDecelNormal.get());
            kStandardLimiter.angularTopSpeed = RadiansPerSecond.of(turnSpeedNormal.get());
            kStandardLimiter.angularAcceleration = RadiansPerSecondPerSecond.of(turnAccelNormal.get());
            kStandardLimiter.angularDeceleration = RadiansPerSecondPerSecond.of(turnDecelNormal.get());
        }

        if (driveSpeedTippy.hasChanged(hash) || driveAccelTippy.hasChanged(hash) || driveDecelTippy.hasChanged(hash)
                || turnSpeedTippy.hasChanged(hash) || turnAccelTippy.hasChanged(hash) || turnDecelTippy.hasChanged(hash)) {
            kTippyLimiter.linearTopSpeed = MetersPerSecond.of(driveSpeedTippy.get());
            kTippyLimiter.linearAcceleration = MetersPerSecondPerSecond.of(driveAccelTippy.get());
            kTippyLimiter.linearDeceleration = MetersPerSecondPerSecond.of(driveDecelTippy.get());
            kTippyLimiter.angularTopSpeed = RadiansPerSecond.of(turnSpeedTippy.get());
            kTippyLimiter.angularAcceleration = RadiansPerSecondPerSecond.of(turnAccelTippy.get());
            kTippyLimiter.angularDeceleration = RadiansPerSecondPerSecond.of(turnDecelTippy.get());
        }
    }
}
