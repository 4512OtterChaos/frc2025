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
import frc.robot.util.FieldUtil.ReefPosition;
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
    
    private final TunableNumber reefAlignXOffset = new TunableNumber("Align/reefAlignXOffset", Units.inchesToMeters(1));
    private Transform2d reefAlignOffset = new Transform2d(reefAlignXOffset.get(), 0, Rotation2d.kZero);

    private final TunableNumber netAlgaeReleaseHeight = new TunableNumber("Commands/algaeShotElevHeightOffsetInches", 12);

    public void periodic() {
        changeTunable();

        adjustDriving();

        viz.update(elevator.getHeight(), manipulator.getPosition(), funnel.getPosition());
    }

    //########## Alignnment Commands

    public Command autoAlign(ReefPosition pos, boolean simpleAlign, boolean forever) {
        List<Pose2d> possibleGoalPoses;
        switch (pos) {
            case LEFT -> possibleGoalPoses = kReefLeftCoralPoses;
            case RIGHT -> possibleGoalPoses = kReefRightCoralPoses;
            default -> possibleGoalPoses = kReefCenterPoses;
        }

        return autoAlign(() -> swerve.getGlobalPoseEstimate().nearest(possibleGoalPoses).plus(reefAlignOffset), simpleAlign, forever, 1)
                .withName((simpleAlign ? "Simple": "") + "AlignToReef" + pos.toString());
    }

    public Command autoAlign(Supplier<Pose2d> goalSupplier, boolean simpleAlign, boolean forever, double slowDistMeters) {
        Supplier<Pose2d> adjGoalSupplier = () -> {
                var curr = swerve.getGlobalPoseEstimate();
                var goal = goalSupplier.get();
                return adjustAlignPoseSlow(goal, curr, slowDistMeters);
        };
        Command command;
        if (simpleAlign) {
            command = swerve.simpleAlignToPose(
                adjGoalSupplier,
                forever
            );
        }
        else {
            command = swerve.alignToPose(
                adjGoalSupplier,
                false,
                forever
            );
        }
        return command.withName((simpleAlign ? "Simple": "") + "AlignTo"+adjGoalSupplier.get().toString());
    }

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
            elevator.setMinC(),
            parallel(
                elevator.setL4C(),
                sequence(
                    waitUntil(()->elevator.getHeight().in(Meters) >= ElevatorConstants.kL4Height.minus(Inches.of(netAlgaeReleaseHeight.get())).in(Meters)),
                    manipulator.scoreAlgaeC().withTimeout(0.5)
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

    public Command autoScore(ReefPosition pos, ElevatorHeight scorePos) {
        Trigger closingInOnGoal = new Trigger(() -> {
            Pose2d swervePose = swerve.getGlobalPoseEstimate();
            Pose2d goalPose = swerve.getGoalPose();
            double dist = goalPose.getTranslation().getDistance(swervePose.getTranslation());
            return dist < 1;
        }).and(swerve.isAligning());

        List<Pose2d> possibleGoalPoses;
        Command elevatorCommand;
        switch (pos) {
            case LEFT -> possibleGoalPoses = kReefLeftCoralPoses;
            case RIGHT -> possibleGoalPoses = kReefRightCoralPoses;
            default -> possibleGoalPoses = kReefCenterPoses;
        }

        switch (scorePos) {
            case L1 -> elevatorCommand = elevator.setL1C();
            case L2 -> elevatorCommand = elevator.setL2C();
            case L3 -> elevatorCommand = elevator.setL3C();
            case L4 -> elevatorCommand = elevator.setL4C();
            default -> elevatorCommand = elevator.setL4C();
        }

        return sequence(
            parallel(
                autoAlign(() -> swerve.getGlobalPoseEstimate().nearest(possibleGoalPoses).plus(reefAlignOffset), false, false, 1),
                sequence(
                    waitUntil(closingInOnGoal),
                    elevatorCommand
                )
            ),
            manipulator.scoreCoralC().asProxy().withTimeout(0.4)
        ).withName( "AlignToReef" + pos.toString() + "AndScore" + scorePos.toString());
    }

    public Command autoCoralStation(CoralStation coralStation){
        Trigger simSkipCoral = new Trigger(() -> {
            Pose2d swervePose = swerve.getGlobalPoseEstimate();
            Pose2d goalPose = swerve.getGoalPose();
            double dist = goalPose.getTranslation().getDistance(swervePose.getTranslation());
            return dist < 0.1 && Robot.isSimulation();
        }).debounce(0.5).and(swerve.isAligning());

        return autoAlign(()->coralStation.getPose().plus(new Transform2d(kRobotLength.div(2).in(Meters), 0, Rotation2d.kZero)), false, true, 1.5)
            .until(manipulator.isCoralDetected().or(funnel.isCoralDetected).or(simSkipCoral));
    }

    public void adjustDriving() {
        double elevatorPercentTravel = elevator.getHeight().div(ElevatorConstants.kMaxTravel).magnitude();

        swerve.driveSpeed = MetersPerSecond.of(elevatorPercentTravel*(driveSpeedTippy.get() - driveSpeedNormal.get()) + driveSpeedNormal.get());
        swerve.turnSpeed = RadiansPerSecond.of(elevatorPercentTravel*(turnSpeedTippy.get() - turnSpeedNormal.get()) + turnSpeedNormal.get());

        swerve.limiter.linearAcceleration = elevatorPercentTravel*(driveAccelTippy.get() - driveAccelNormal.get()) + driveAccelNormal.get();
        swerve.limiter.linearDeceleration = elevatorPercentTravel*(driveDecelTippy.get() - driveDecelNormal.get()) + driveDecelNormal.get();
        swerve.limiter.angularAcceleration = elevatorPercentTravel*(turnAccelTippy.get() - turnAccelNormal.get()) + turnAccelNormal.get();
        swerve.limiter.angularDeceleration = elevatorPercentTravel*(turnDecelTippy.get() - turnDecelNormal.get()) + turnDecelNormal.get();
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

        reefAlignXOffset.poll();

        netAlgaeReleaseHeight.poll();
        
        int hash = hashCode();
        if (reefAlignXOffset.hasChanged(hash)) {
            reefAlignOffset = new Transform2d(reefAlignXOffset.get(), 0, Rotation2d.kZero);
        }
    }
}
