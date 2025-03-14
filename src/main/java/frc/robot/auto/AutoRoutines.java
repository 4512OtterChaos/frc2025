package frc.robot.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class AutoRoutines {
    private final AutoFactory m_factory;

    private final CommandSwerveDrivetrain swerve;
    private final Elevator elevator;
    private final Manipulator manipulator;

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain swerve, Elevator elevator, Manipulator manipulator) {
        m_factory = factory;

        this.swerve = swerve;
        this.elevator = elevator;
        this.manipulator = manipulator;
    }

    public Command taxiAuto() {
        return sequence(
            runOnce(()->swerve.resetRotation(Rotation2d.k180deg), swerve),
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

    public Command middle1CoralL1() {
        return sequence(
            runOnce(()->swerve.resetPose(new Pose2d(7.5, 4.2, Rotation2d.k180deg)), swerve),
            swerve.drive(()->new ChassisSpeeds(-1.5, 0, 0)).withTimeout(1),
            swerve.drive(()->new ChassisSpeeds(-0.5, 0, 0)).withTimeout(1.5),
            swerve.stop(),
            elevator.setL1C().withTimeout(2),
            manipulator.setVoltageScoreC().withTimeout(2)
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

    // public Command middle1Coral() {
    //     return sequence(
    //         runOnce(()->drivetrain.resetPose(new Pose2d(7.5, 4.2, Rotation2d.k180deg)), drivetrain)
    //         // align to reef pose,
    //         // score L1
    //     );
    // }
}