package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.util.FieldUtil.kReefPoleDist;
import static frc.robot.util.FieldUtil.kReefTrl;
import static frc.robot.util.FieldUtil.kReefWidth;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.OCXboxController;
import frc.robot.util.TunableNumber;


public class Superstructure {
    private CommandSwerveDrivetrain drive;
    private Manipulator manipulator;
    private Elevator elevator;

    private OCXboxController driver;

    public Superstructure(CommandSwerveDrivetrain drive, Manipulator manipulator, Elevator elevator, OCXboxController driver) {
        this.drive = drive;
        this.manipulator = manipulator;
        this.elevator = elevator;
        this.driver = driver;
    }

    
    public static final Distance kRobotWidth = Inches.of(33.875);

    HolonomicDriveController driveController;

    SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
        // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors //TODO: Is this right?

        private TunableNumber pathDriveKP = new TunableNumber("Path/pathDriveKP", TunerConstants.pathSteerGains.kP);
    private TunableNumber pathDriveKI = new TunableNumber("Path/pathDriveKI", TunerConstants.pathSteerGains.kI);
    private TunableNumber pathDriveKD = new TunableNumber("Path/pathDriveKD", TunerConstants.pathSteerGains.kD);
    
    private TunableNumber pathSteerKP = new TunableNumber("Path/pathSteerKP", TunerConstants.pathSteerGains.kP);
    private TunableNumber pathSteerKI = new TunableNumber("Path/pathSteerKI", TunerConstants.pathSteerGains.kI);
    private TunableNumber pathSteerKD = new TunableNumber("Path/pathSteerKD", TunerConstants.pathSteerGains.kD);

    private TunableNumber pathSpeed = new TunableNumber("Path/pathSpeed", 3);

    
    private static final Translation2d kCoralScoreLeftPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(kRobotWidth.div(2)).in(Meters)),
        kReefTrl.getMeasureY()/*.plus(kReefPoleDist.div(2))*/.in(Meters));

    
    private static final Translation2d kCoralScoreRightPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(kRobotWidth.div(2)).in(Meters)),
        kReefTrl.getMeasureY().minus(kReefPoleDist.div(2)).in(Meters));
    
    public static final List<Pose2d> kCoralScoringPositions = new ArrayList<Pose2d>() {{
        add(new Pose2d(kCoralScoreLeftPoseTemplate, Rotation2d.fromDegrees(0)));
        // add(new Pose2d(kCoralScoreRightPoseTemplate, Rotation2d.fromDegrees(0)));

        // add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        // add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));

        // add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        // add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));

        // add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        // add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));

        // add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        // add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));

        // add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
        // add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};










    public void periodic() {
        if (!(elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.plus(ElevatorConstants.kHeightTolerance).in(Meters))){
            updateDriveSpeed(driver);
        }
        tipProtection();
        changeTunable();
    }

    public Command intake(){
        return sequence(
            elevator.setHeightC(ElevatorConstants.kMinHeight).withTimeout(0.75).unless(()->elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.in(Inches)),
            manipulator.setVoltageInC()
        );
    }

    // public Command outtake(){
    //     return sequence(
    //         elevator.setHeightC(ElevatorConstants.kMinHeight).withTimeout(0.75).unless(()->elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.in(Inches)),
    //         manipulator.setVoltageOutC()
    //     );
    // }

    public Command driveToScorePointVector(){
        Pose2d currentPose = drive.getState().Pose;
        Pose2d targetPose = currentPose.nearest(kCoralScoringPositions);

        Translation2d translationError = drive.getState().Pose.relativeTo(targetPose).getTranslation();
        Rotation2d rotError = drive.getState().Pose.relativeTo(targetPose).getRotation();
        Distance straightDistError = Meters.of(Math.sqrt(Math.pow(translationError.getX(), 2) + Math.pow(translationError.getY(), 2)));

        double desiredRotSpeed = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
        // double desiredRotAcceleration = RotationsPerSecondPerSecond.of(3).in(RadiansPerSecondPerSecond);

        double desiredSpeed;
        double xSpeed;
        double ySpeed;
        double rotSpeed;

        Translation2d adjustedError = translationError;

        //TODO: adjustPose for y and rott offset

        desiredSpeed = straightDistError.in(Meters)*.5;
        MathUtil.clamp(desiredSpeed, 0, pathSpeed.get());
        
        rotSpeed = rotError.getRotations()*3;
        MathUtil.clamp(rotSpeed, 0, desiredRotSpeed);

        xSpeed = desiredSpeed * adjustedError.getX()/straightDistError.in(Meters);
        ySpeed = desiredSpeed * adjustedError.getY()/straightDistError.in(Meters);
        
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        return drive.applyRequest(()->robotSpeeds.withSpeeds(targetSpeeds))/*.until(()->driveController.atReference()).repeatedly()*/;
    }

    public Command driveToScorePoint(){
        return drive.applyRequest(()->robotSpeeds.withSpeeds(chassisSpeedsToScorePoint()))/*.until(()->driveController.atReference()).repeatedly()*/;
    }

    private ChassisSpeeds chassisSpeedsToScorePoint(){    
        Pose2d currentPose = drive.getState().Pose;
        Pose2d targetPose = currentPose.nearest(kCoralScoringPositions);

        double desiredSpeed = pathSpeed.get();
        double desiredRotSpeed = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
        double desiredRotAcceleration = RotationsPerSecondPerSecond.of(3).in(RadiansPerSecondPerSecond);
        
        PIDController xController = new PIDController(pathDriveKP.get(), pathDriveKI.get(), pathDriveKD.get());
        ProfiledPIDController thetaController = new ProfiledPIDController(pathSteerKP.get(), pathSteerKI.get(), pathSteerKD.get(), new Constraints(desiredRotSpeed, desiredRotAcceleration));

        driveController = new HolonomicDriveController(xController, xController, thetaController);
        driveController.setTolerance(new Pose2d(.2,.2, Rotation2d.fromDegrees(5)));
        
    
        return driveController.calculate(currentPose, targetPose, desiredSpeed, targetPose.getRotation()); 
    }

    public void updateDriveSpeed(OCXboxController controller){//TODO:Just like make it work/do it
        Distance currentElevatorTravel = Meters.of(elevator.getElevatorHeightMeters()).minus(ElevatorConstants.kMinHeight);
        Distance maxElevatorTravel = ElevatorConstants.kMaxHeight.minus(ElevatorConstants.kMinHeight);
        double elevatorPercentTravel = currentElevatorTravel.in(Meters) / maxElevatorTravel.in(Meters);

        double speedDifference = OCXboxController.kSpeedDefault.get() - OCXboxController.kSpeedSlow.get();

        controller.setDriveSpeed(OCXboxController.kSpeedDefault.get() - (elevatorPercentTravel * speedDifference));

    }

    public void tipProtection(){
        double rollRadians = Math.abs(drive.getRotation3d().getX());
        double pitchRadians = Math.abs(drive.getRotation3d().getY());
        double angleTolerance = ElevatorConstants.kTipAngleTolerance.in(Radians);

        if ((rollRadians >= angleTolerance) || (pitchRadians >= angleTolerance)){
            elevator.setHeightC(ElevatorConstants.kMinHeight);
        }
    }

    public Command coralNoSlide(){
        return run(()->{
            if (elevator.getElevatorHeightMeters() < elevator.getTargetMeters() && !elevator.isWithinTolerance()){
                manipulator.setVoltage(-1);
            }
        }, manipulator);
    }

    private void changeTunable() {
        pathDriveKP.poll();
        pathDriveKI.poll();
        pathDriveKD.poll();

        pathSteerKP.poll();
        pathSteerKI.poll();
        pathSteerKD.poll();

        pathSpeed.poll();
    }
}
