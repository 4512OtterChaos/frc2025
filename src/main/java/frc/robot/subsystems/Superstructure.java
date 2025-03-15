package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.FieldUtil.*;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.FieldUtil.ReefPosition;
import frc.robot.util.TunableNumber;


public class Superstructure {
    private CommandSwerveDrivetrain swerve;
    private Manipulator manipulator;
    private Elevator elevator;

    private SuperstructureViz viz = new SuperstructureViz();

    public Superstructure(CommandSwerveDrivetrain drive, Manipulator manipulator, Elevator elevator) {
        this.swerve = drive;
        this.manipulator = manipulator;
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
    
    private final TunableNumber reefAlignXOffset = new TunableNumber("Align/reefAlignXOffset", 0);
    private Transform2d reefAlignOffset = new Transform2d();

    public void periodic() {
        changeTunable();

        adjustDriving();

        viz.update(elevator.getHeight(), manipulator.getPosition());
    }

    //########## Alignnment Commands

    public Command autoAlign(ReefPosition pos, boolean simpleAlign, boolean forever) {
        List<Pose2d> possibleGoalPoses;
        switch (pos) {
            case LEFT -> possibleGoalPoses = kReefLeftCoralPoses;
            case RIGHT -> possibleGoalPoses = kReefRightCoralPoses;
            default -> possibleGoalPoses = kReefCenterPoses;
        }

        return autoAlign(() -> swerve.getGlobalPoseEstimate().nearest(possibleGoalPoses).plus(reefAlignOffset), simpleAlign, forever)
                .withName((simpleAlign ? "Simple": "") + "AlignToReef" + pos.toString());
    }

    public Command autoAlign(Supplier<Pose2d> goalSupplier, boolean simpleAlign, boolean forever) {
        Supplier<Pose2d> adjGoalSupplier = () -> {
            var curr = swerve.getGlobalPoseEstimate();
                var goal = goalSupplier.get();
                // return adjustAlignPose(goal, curr);
                return adjustAlignPoseSlow(goal, curr);
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

    public Pose2d adjustAlignPoseSlow(Pose2d goalPose, Pose2d currentPose) {
        var currRelToGoal = currentPose.relativeTo(goalPose);
        double angularErrorRots = Math.abs(currRelToGoal.getRotation().getRotations());
        double trlErrorMeters = currRelToGoal.getTranslation().getNorm();

        double trlAngleToGoalError = Math.abs(currRelToGoal.getTranslation().getAngle().plus(Rotation2d.kPi).getRotations());

        double xOffsetAngular = angularErrorRots * 2;
        double xOffsetTrlY = Math.min(trlAngleToGoalError * trlErrorMeters * 7, 1);

        double xOffset = Math.max(xOffsetAngular, xOffsetTrlY);
        var adjGoal = goalPose.plus(new Transform2d(-xOffset, 0, Rotation2d.kZero));
        if (trlErrorMeters < 1) {
            adjGoal = currentPose.interpolate(adjGoal, Math.max(0.1 / trlErrorMeters, trlErrorMeters / 2)); // idk
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
            elevator.setL4C().until(()->elevator.getHeight().in(Meters) >= ElevatorConstants.kL4Height.minus(Inches.of(12)).in(Meters)),
            manipulator.scoreAlgaeC().withTimeout(1)
        ).withName("AlgeaShoot");
    }

    // public Command algaeShootVelocity(){
    //     return sequence(
    //         elevator.setMinC(),
    //         elevator.setL4C().until(()->elevator.getHeight().in(Meters) >= ElevatorConstants.kL4Height.minus(Inches.of(12)).in(Meters)),
    //         manipulator.algaeShootVelocity()
    //     ).withName("AlgeaShootVelocity");
    // }

    public void adjustDriving() {
        double elevatorPercentTravel = elevator.getHeight().div(ElevatorConstants.kMaxTravel).magnitude();

        swerve.driveSpeed = MetersPerSecond.of(elevatorPercentTravel*(driveSpeedTippy.get() - driveSpeedNormal.get()) + driveSpeedNormal.get());
        swerve.turnSpeed = RadiansPerSecond.of(elevatorPercentTravel*(turnSpeedTippy.get() - turnSpeedNormal.get()) + turnSpeedNormal.get());

        swerve.limiter.linearAcceleration = elevatorPercentTravel*(driveAccelTippy.get() - driveAccelNormal.get()) + driveAccelNormal.get();
        swerve.limiter.linearDeceleration = elevatorPercentTravel*(driveDecelTippy.get() - driveDecelNormal.get()) + driveDecelNormal.get();
        swerve.limiter.angularAcceleration = elevatorPercentTravel*(turnAccelTippy.get() - turnAccelNormal.get()) + turnAccelNormal.get();
        swerve.limiter.angularDeceleration = elevatorPercentTravel*(turnDecelTippy.get() - turnDecelNormal.get()) + turnDecelNormal.get();
    }

    // public void tipProtection(){
    //     double rollRadians = Math.abs(swerve.getRotation3d().getX());
    //     double pitchRadians = Math.abs(swerve.getRotation3d().getY());
    //     double angleTolerance = ElevatorConstants.kTipAngleTolerance.in(Radians);

    //     if ((rollRadians >= angleTolerance) || (pitchRadians >= angleTolerance)){
    //         elevator.setMinC(); // this should be on a Trigger
    //     }
    // }

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
        
        int hash = hashCode();
        if (reefAlignXOffset.hasChanged(hash)) {
            reefAlignOffset = new Transform2d(reefAlignXOffset.get(), 0, Rotation2d.kZero);
        }
    }
}
