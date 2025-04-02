package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.kRobotWidth;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.CoralStation;
import frc.robot.util.FieldUtil.ReefFace;
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
            waitSeconds(4),
            // Score on far left/right right branch
            superstructure.autoScore(ReefFace.BACK, Alignment.RIGHT, ElevatorHeight.L4)
        );
    }

    public Command Middle1CoralL41Algae() {
        Pose2d startingPose = new Pose2d(7.3, FieldUtil.kFieldWidth.div(2).in(Meters), Rotation2d.k180deg);
        Pose2d algaePrepPose = new Pose2d(7.3, FieldUtil.kFieldWidth.div(2).plus(kRobotWidth.times(2)).in(Meters), Rotation2d.kZero); //TODO: use REAL pose
        
        return sequence(
            // Reset odom
            runOnce(()->swerve.resetPose(startingPose), swerve),
            waitSeconds(4),
            // Score on far left/right right branch
            superstructure.autoScore(ReefFace.BACK, Alignment.RIGHT, ElevatorHeight.L4),

            //Back up then pick up algae
            swerve.drive(()->new ChassisSpeeds(-1, 0, 0)).withTimeout(0.5),
            superstructure.autoAlgaePickUp(ReefFace.BACK),

            //TODO: align to barge score pose
            swerve.alignToReef(()-> algaePrepPose, false), //TODO: transfer to align barge command(?)
            
            //Shoot the algae
            superstructure.algaeShoot()
        );
    }

    public Command Middle1CoralL42Algae() {
        Pose2d startingPose = new Pose2d(7.3, FieldUtil.kFieldWidth.div(2).in(Meters), Rotation2d.k180deg);
        Pose2d algaePrepPose = new Pose2d(7.3, FieldUtil.kFieldWidth.div(2).plus(kRobotWidth.times(2)).in(Meters), Rotation2d.kZero); //TODO: use REAL pose
        Pose2d algaePrepPose2 = algaePrepPose.plus(new Transform2d(Meters.of(0), Meters.of(0.5), Rotation2d.kZero)); //TODO: use REAL pose
        
        return sequence(
            // Reset odom
            runOnce(()->swerve.resetPose(startingPose), swerve),
            waitSeconds(4),
            // Score on far left/right right branch
            superstructure.autoScore(ReefFace.BACK, Alignment.RIGHT, ElevatorHeight.L4),

            //Back up then pick up algae
            swerve.drive(()->new ChassisSpeeds(-1, 0, 0)).withTimeout(0.5),
            superstructure.autoAlgaePickUp(ReefFace.BACK),

            //align to barge score pose
            swerve.alignToReef(()-> algaePrepPose, false), //TODO: transfer to align barge command(?)
            
            //Shoot the algae
            superstructure.algaeShoot(),

            
            superstructure.autoAlgaePickUp(ReefFace.FARLEFT),

            //align to barge score pose
            swerve.alignToReef(()-> algaePrepPose2, false), //TODO: transfer to align barge command(?)
            
            //Shoot the algae
            superstructure.algaeShoot()
        );
    }

    public Command Wall3CoralL4(boolean rightSide) {
        Pose2d startLeftPose = new Pose2d(7.3, FieldUtil.kFieldWidth.minus(kRobotWidth.div(2)).in(Meters), Rotation2d.kCW_90deg);
        Pose2d startRightPose = FieldUtil.mirrorY(startLeftPose);
        Pose2d startPose = rightSide ? startRightPose : startLeftPose;

        CoralStation coralStation = rightSide ? CoralStation.RIGHT : CoralStation.LEFT;
        Alignment stationAlignment = rightSide ? Alignment.RIGHT : Alignment.LEFT;

        ReefFace reefFace1 = rightSide ? ReefFace.FARRIGHT : ReefFace.FARLEFT;
        Alignment reefAlign1 = rightSide ? Alignment.LEFT : Alignment.RIGHT;
        ReefFace reefFace2 = rightSide ? ReefFace.NEARRIGHT : ReefFace.NEARLEFT;
        Alignment reefAlign2 = rightSide ? Alignment.RIGHT : Alignment.LEFT;
        ReefFace reefFace3 = rightSide ? ReefFace.NEARRIGHT : ReefFace.NEARLEFT;
        Alignment reefAlign3 = reefAlign1;

        return sequence(
            // Reset odom
            runOnce(()->swerve.resetPose(startPose), swerve),
            // Score on far left/right right branch
            superstructure.autoScore(reefFace1, reefAlign1, ElevatorHeight.L4),
            // Drive to coral station and wait for coral
            parallel(
                sequence(
                    swerve.drive(()->new ChassisSpeeds(0, rightSide ? -1 : 1, 0)).withTimeout(0.5),
                    superstructure.autoCoralStation(coralStation, stationAlignment)
                ),
                sequence(
                    waitSeconds(0.1),
                    elevator.setMinC()
                )
            ),
            // Score on close left/right
            superstructure.autoScore(reefFace2, reefAlign2, ElevatorHeight.L4),
            // Drive to coral station and wait for coral
            parallel(
                superstructure.autoCoralStation(coralStation, stationAlignment),
                sequence(
                    waitSeconds(0.1),
                    elevator.setMinC()
                )
            ),
            superstructure.autoScore(reefFace3, reefAlign3, ElevatorHeight.L4),
            // Drive to coral station and wait for coral
            parallel(
                superstructure.autoCoralStation(coralStation, stationAlignment),
                sequence(
                    waitSeconds(0.1),
                    elevator.setMinC()
                )
            )
        );
    }
}