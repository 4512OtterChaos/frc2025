package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.FieldUtil.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
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
    
    HolonomicDriveController driveController;

    SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
        // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors //TODO: Is this right?

    // private TunableNumber pathSpeed = new TunableNumber("Path/pathSpeed", 3);

    
    private static final Translation2d kCoralScoreLeftPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(kRobotLength/*.div(2) */).in(Meters)),
        kReefTrl.getMeasureY().plus(kReefPoleDist.div(2)).in(Meters));

    
    private static final Translation2d kCoralScoreRightPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(kRobotLength/*.div(2) */).in(Meters)),
        kReefTrl.getMeasureY().minus(kReefPoleDist.div(2)).in(Meters));

    public static final List<Pose2d> kLeftCoralScoringPositions = new ArrayList<Pose2d>() {{
        add(new Pose2d(kCoralScoreLeftPoseTemplate, Rotation2d.fromDegrees(0)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};

    public static final List<Pose2d> kRightCoralScoringPositions = new ArrayList<Pose2d>() {{
        add(new Pose2d(kCoralScoreRightPoseTemplate, Rotation2d.fromDegrees(0)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};
    
    public static final List<Pose2d> kCoralScoringPositions = new ArrayList<Pose2d>() {{
        add(new Pose2d(kCoralScoreLeftPoseTemplate, Rotation2d.fromDegrees(0)));
        add(new Pose2d(kCoralScoreRightPoseTemplate, Rotation2d.fromDegrees(0)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};

    public void periodic() {
        changeTunable();

        adjustDriving();

        viz.update(elevator.getHeight(), manipulator.getPosition());
    }

    // public Command driveToScorePointVector(){ //TODO: add accel limiter
    //     Pose2d currentPose = drive.getState().Pose;
    //     Pose2d targetPose = currentPose.nearest(kCoralScoringPositions);

    //     Translation2d translationError = drive.getState().Pose.relativeTo(targetPose).getTranslation();
    //     Rotation2d rotError = drive.getState().Pose.relativeTo(targetPose).getRotation();
    //     Distance straightDistError = Meters.of(Math.sqrt(Math.pow(translationError.getX(), 2) + Math.pow(translationError.getY(), 2)));

    //     double desiredRotSpeed = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
    //     // double desiredRotAcceleration = RotationsPerSecondPerSecond.of(3).in(RadiansPerSecondPerSecond);

    //     double desiredSpeed;
    //     double xSpeed;
    //     double ySpeed;
    //     double rotSpeed;

    //     Translation2d adjustedError = translationError;

    //     //TODO: adjustPose for y and rott offset

    //     desiredSpeed = straightDistError.in(Meters)*.5;
    //     MathUtil.clamp(desiredSpeed, 0, pathSpeed.get());
        
    //     rotSpeed = rotError.getRotations()*3;
    //     MathUtil.clamp(rotSpeed, 0, desiredRotSpeed);

    //     xSpeed = desiredSpeed * adjustedError.getX()/straightDistError.in(Meters);
    //     ySpeed = desiredSpeed * adjustedError.getY()/straightDistError.in(Meters);
        
    //     ChassisSpeeds targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

    //     return drive.applyRequest(()->robotSpeeds.withSpeeds(targetSpeeds))/*.until(()->driveController.atReference()).repeatedly()*/;
    // }


    //Alignnment Commands

    public Command autoAlignToReef(){
        return swerve.driveToPose(() -> swerve.getVisionEstimatedPose().nearest(kCoralScoringPositions));
        // return swerve.applyRequest(()->robotSpeeds.withSpeeds(chassisSpeedsToScorePoint()))/*.until(()->driveController.atReference()).repeatedly()*/;
    }

    public Command autoAlignLeft(){
        // return swerve.driveToPose(() -> swerve.getState().Pose.nearest(kLeftCoralScoringPositions));
        return swerve.driveToPose(() -> {
            var curr = swerve.getVisionEstimatedPose();
            var goal = curr.nearest(kLeftCoralScoringPositions);
            return adjustAlignPose(goal, curr);
        });
        // return swerve.applyRequest(()->robotSpeeds.withSpeeds(chassisSpeedsToScorePoint()))/*.until(()->driveController.atReference()).repeatedly()*/;
    }

    public Command autoAlignRight(){
        // return swerve.driveToPose(() -> swerve.getState().Pose.nearest(kRightCoralScoringPositions));
        return swerve.driveToPose(() -> {
            var curr = swerve.getVisionEstimatedPose();
            var goal = curr.nearest(kRightCoralScoringPositions);
            return adjustAlignPose(goal, curr);
        });
        // return swerve.applyRequest(()->robotSpeeds.withSpeeds(chassisSpeedsToScorePoint()))/*.until(()->driveController.atReference()).repeatedly()*/;
    }

    public Pose2d adjustAlignPose(Pose2d goalPose, Pose2d currentPose) {
        var currRelToGoal = currentPose.relativeTo(goalPose);
        double angularErrorRots = Math.abs(currRelToGoal.getRotation().getRotations());
        double trlErrorMeters = currRelToGoal.getTranslation().getNorm();

        if ((trlErrorMeters < kAlignmentDrivePosTol) && (angularErrorRots < kAlignmentTurnPosTol)){
            return goalPose;
        }

        double trlAngleToGoalError = Math.abs(currRelToGoal.getTranslation().getAngle().plus(Rotation2d.kPi).getRotations());

        double xOffsetAngular = angularErrorRots * 2;
        double xOffsetTrlY = Math.min(trlAngleToGoalError * trlErrorMeters * 5, 1);

        double xOffset = Math.max(xOffsetAngular, xOffsetTrlY);
        return goalPose.plus(new Transform2d(-xOffset, 0, Rotation2d.kZero));
    }

    // private Command driveToScorePointFancy(){
    //     return swerve.driveToPose(() ->{
    //         Pose2d scorePose = swerve.getState().Pose.nearest(kCoralScoringPositions);
    //         Pose2d adjustedPose = scorePose.relativeTo();
    //     });
    //     // return swerve.applyRequest(()->robotSpeeds.withSpeeds(chassisSpeedsToScorePoint()))/*.until(()->driveController.atReference()).repeatedly()*/;
    // }

    // private ChassisSpeeds chassisSpeedsToScorePoint(){    
    //     Pose2d currentPose = drive.getState().Pose;
    //     Pose2d targetPose = currentPose.nearest(kCoralScoringPositions);

    //     double desiredSpeed = pathSpeed.get();
    //     double desiredRotSpeed = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
    //     double desiredRotAcceleration = RotationsPerSecondPerSecond.of(3).in(RadiansPerSecondPerSecond);
        
    //     PIDController xController = new PIDController(pathDriveKP.get(), pathDriveKI.get(), pathDriveKD.get());
    //     ProfiledPIDController thetaController = new ProfiledPIDController(pathSteerKP.get(), pathSteerKI.get(), pathSteerKD.get(), new Constraints(desiredRotSpeed, desiredRotAcceleration));

    //     driveController = new HolonomicDriveController(xController, xController, thetaController);
    //     driveController.setTolerance(new Pose2d(.2,.2, Rotation2d.fromDegrees(5)));
        
    
    //     return driveController.calculate(currentPose, targetPose, desiredSpeed, targetPose.getRotation()); 
    // }

    public Command algeaShoot(){
        return sequence(
            elevator.setMinC(),
            elevator.setL4C().until(()->elevator.getHeight().in(Meters) >= ElevatorConstants.kL4Height.minus(Inches.of(12)).in(Meters)),
            manipulator.algaeShoot()
        ).withName("AlgeaShoot");
    }

    public Command algeaShootVelocity(){
        return sequence(
            elevator.setMinC(),
            elevator.setL4C().until(()->elevator.getHeight().in(Meters) >= ElevatorConstants.kL4Height.minus(Inches.of(12)).in(Meters)),
            manipulator.algaeShootVelocity()
        ).withName("AlgeaShootVelocity");
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

        // pathSpeed.poll();
    }
}
