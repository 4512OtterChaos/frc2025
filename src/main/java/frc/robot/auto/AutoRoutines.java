package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.kRobotWidth;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.CoralStation;
import frc.robot.util.FieldUtil.Alignment;

public class AutoRoutines {
    private final Superstructure superstructure;
    private final CommandSwerveDrivetrain swerve;
    private final Elevator elevator;
    private final Manipulator manipulator;

    public AutoRoutines(Superstructure superstructure, CommandSwerveDrivetrain swerve, Elevator elevator, Manipulator manipulator) {

        this.superstructure = superstructure;
        this.swerve = swerve;
        this.elevator = elevator;
        this.manipulator = manipulator;
    }

    public Command taxiAuto() {
        return sequence(
            runOnce(()->swerve.resetPose(new Pose2d(FieldUtil.kFieldWidth, FieldUtil.kFieldWidth, Rotation2d.k180deg)), swerve),
            swerve.drive(()->new ChassisSpeeds(-1, 0, 0)).withTimeout(1.5),
            swerve.stop()
        );
    }

    public Command taxiFar() {
        return sequence(
            runOnce(()->swerve.resetPose(new Pose2d(7.5, 4.2, Rotation2d.k180deg)), swerve),
            swerve.drive(()->new ChassisSpeeds(-1.5, 0, 0)).withTimeout(1),
            swerve.drive(()->new ChassisSpeeds(-0.5, 0, 0)).withTimeout(1.5),
            swerve.stop()
        );
    }

    public Command Middle1CoralL4() {
        return sequence(
            // Reset odom
            runOnce(()->swerve.resetPose(new Pose2d(7.5, 4.2, Rotation2d.k180deg)), swerve),
            waitSeconds(1),
            // Score on far left/right right branch
            superstructure.autoScore(Alignment.RIGHT, ElevatorHeight.L4)
        );
    }

    public Command Wall3CoralL4(boolean rightSide) {
        Pose2d startLeftPose = new Pose2d(7.3, FieldUtil.kFieldWidth.minus(kRobotWidth.div(2)).in(Meters), Rotation2d.kCW_90deg);
        Pose2d startRightPose = FieldUtil.mirrorY(startLeftPose);
        Pose2d startPose = rightSide ? startRightPose : startLeftPose;

        CoralStation coralStation = rightSide ? CoralStation.RIGHT : CoralStation.LEFT;

        Pose2d preAlign1Left = new Pose2d(3, 6.5, Rotation2d.fromDegrees(-60));
        Pose2d preAlign1Right = FieldUtil.mirrorY(preAlign1Left);
        Pose2d preAlign1 = rightSide ? preAlign1Right : preAlign1Left;

        Alignment reefPos1 = rightSide ? Alignment.LEFT : Alignment.RIGHT;
        Alignment reefPos2 = rightSide ? Alignment.RIGHT : Alignment.LEFT;
        Alignment reefPos3 = reefPos1;

        return sequence(
            // Reset odom
            runOnce(()->swerve.resetPose(startPose), swerve),
            // Score on far left/right right branch
            superstructure.autoScore(reefPos1, ElevatorHeight.L4),
            // Drive to coral station and wait for coral
            parallel(
                sequence(
                    swerve.drive(()->new ChassisSpeeds(0, rightSide ? -1 : 1, 0)).withTimeout(0.5),
                    superstructure.autoCoralStation(coralStation, Alignment.LEFT)
                ),
                sequence(
                    waitSeconds(0.1),
                    elevator.setMinC()
                )
            ),
            // Score on close left/right
            // swerve.alignToPose(()->preAlign1, 0.5, 1, 1, 1, false, false),
            superstructure.autoScore(reefPos2, ElevatorHeight.L4),
            // Drive to coral station and wait for coral
            parallel(
                superstructure.autoCoralStation(coralStation, Alignment.LEFT),
                sequence(
                    waitSeconds(0.1),
                    elevator.setMinC()
                )
            ),
            // swerve.alignToPose(()->preAlign1, 0.5, 1, 1, 1, false, false),
            superstructure.autoScore(reefPos3, ElevatorHeight.L4)
        );
    }

    // public AutoRoutine simplePathAuto() {
    //     final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
    //     final AutoTrajectory simplePath = routine.trajectory("SimplePath");

    //     routine.active().onTrue(
    //         simplePath.resetOdometry()
    //             .andThen(simplePath.cmd())
    //     );
    //     return routine;p
    // }
}