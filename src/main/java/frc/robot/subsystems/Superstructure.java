package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.util.FieldUtil.kReefPoleDist;
import static frc.robot.util.FieldUtil.kReefTrl;
import static frc.robot.util.FieldUtil.kReefWidth;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveRequest;

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
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.OCXboxController;


public class Superstructure {
    private CommandSwerveDrivetrain drive;
    private Telemetry telemetry;
    private Manipulator manipulator;
    private Elevator elevator;

    private OCXboxController driver;

    public Superstructure(CommandSwerveDrivetrain drive, Telemetry telemetry, Manipulator manipulator, Elevator elevator, OCXboxController driver) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.manipulator = manipulator;
        this.elevator = elevator;
        this.driver = driver;
    }

    
    public static final Distance kRobotWidth = Meters.of(33.875);

    SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
        // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors //TODO: Is this right?

    public static final Translation2d kCoralScoreLeftPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(kRobotWidth.div(2)).in(Meters)),
        kReefTrl.getMeasureY().plus(kReefPoleDist.div(2)).in(Meters));

    
    public static final Translation2d kCoralScoreRightPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(kRobotWidth.div(2)).in(Meters)),
        kReefTrl.getMeasureY().minus(kReefPoleDist.div(2)).in(Meters));
    
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
        if (!(elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.plus(ElevatorConstants.kHeightTolerance).in(Meters))){
            updateDriveSpeed(driver);
        }
        tipProtection();
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

    public Command driveToScorePoint(){
        return drive.applyRequest(()->robotSpeeds.withSpeeds(chassisSpeedsToScorePoint()));
    }

    private ChassisSpeeds chassisSpeedsToScorePoint(){    
        Pose2d currentPose = drive.getState().Pose;
        Pose2d targetPose = currentPose.nearest(kCoralScoringPositions);

        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
        double MaxAngularAcceleration = RotationsPerSecondPerSecond.of(3).in(RadiansPerSecondPerSecond);

        Slot0Configs driveGains = TunerConstants.driveGains;
        Slot0Configs steerGains = TunerConstants.steerGains;
        
        PIDController xController = new PIDController(driveGains.kP, driveGains.kI, driveGains.kD);
        ProfiledPIDController thetaController = new ProfiledPIDController(steerGains.kP, steerGains.kI, steerGains.kD, new Constraints(MaxAngularRate, MaxAngularAcceleration));

        HolonomicDriveController driveController = new HolonomicDriveController(xController, xController, thetaController);
        driveController.setTolerance(new Pose2d(.1,.1, Rotation2d.fromDegrees(3)));

    
        return driveController.calculate(currentPose, targetPose, MaxSpeed, targetPose.getRotation()); 
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
}
