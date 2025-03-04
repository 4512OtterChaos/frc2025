package frc.robot.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class AutoRoutines {
    private final AutoFactory m_factory;

    private final CommandSwerveDrivetrain drivetrain;
    private final Elevator elevator;
    private final Manipulator manipulator;

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain, Elevator elevator, Manipulator manipulator) {
        m_factory = factory;

        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.manipulator = manipulator;
    }

    public Command taxiAuto() {
        return sequence(
            runOnce(()->drivetrain.resetRotation(Rotation2d.k180deg), drivetrain),
            drivetrain.applyRequest(()->new SwerveRequest.FieldCentric().withVelocityX(-1)).withTimeout(1.5),
            runOnce(()->drivetrain.setControl(new SwerveRequest.FieldCentric()), drivetrain)
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

    public Command middle1CoralL1() {
        return sequence(
            runOnce(()->drivetrain.resetPose(new Pose2d(7.5, 4.2, Rotation2d.k180deg)), drivetrain),
            drivetrain.applyRequest(()->new SwerveRequest.FieldCentric().withVelocityX(-1.5)).withTimeout(1),
            drivetrain.applyRequest(()->new SwerveRequest.FieldCentric().withVelocityX(-0.5)).withTimeout(1.5),
            runOnce(()->drivetrain.setControl(new SwerveRequest.FieldCentric()), drivetrain),
            elevator.setL1C().withTimeout(2),
            manipulator.setVoltageOutC().withTimeout(2)
        );
    }

    public Command taxiFar() {
        return sequence(
            runOnce(()->drivetrain.resetPose(new Pose2d(7.5, 4.2, Rotation2d.k180deg)), drivetrain),
            drivetrain.applyRequest(()->new SwerveRequest.FieldCentric().withVelocityX(-1.5)).withTimeout(1),
            drivetrain.applyRequest(()->new SwerveRequest.FieldCentric().withVelocityX(-0.5)).withTimeout(1.5),
            runOnce(()->drivetrain.setControl(new SwerveRequest.FieldCentric()), drivetrain)
        );
    }

    // public Command middle1Coral() {
    //     return sequence(
    //         runOnce(()->drivetrain.resetPose(new Pose2d(7.5, 4.2, Rotation2d.k180deg)), drivetrain)
    //         // align to reef pose,
    //         // score L1
    //     );
    // }
}