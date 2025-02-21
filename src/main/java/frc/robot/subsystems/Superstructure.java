package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import static frc.robot.util.FieldUtil.*;
import frc.robot.util.OCXboxController;


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

    
    public static final Distance kRobotWidth = Meters.of(33.875);

    public static final Translation2d kCoralScorePoseFar = new Translation2d(kReefWidth.plus(kRobotWidth.div(2)).in(Meters), kReefPoleDist.div(2).in(Meters));

    public static final Pose2d[] kCoralScoringPositions = {
        kCoralScorePoseFar,
        kCoralScorePoseFar.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)),
        ,
        ,
        ,

    };

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

    public Command chooseScorePoint(){//TODO:Just like make it work/do it
        return sequence(
            elevator.setHeightC(ElevatorConstants.kMinHeight).withTimeout(0.75).unless(()->elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.in(Inches)),
            manipulator.setVoltageOutC()
        );
    }

    public void updateDriveSpeed(OCXboxController controller){//TODO:Just like make it work/do it
        Distance currentElevatorTravel = Meters.of(elevator.getElevatorHeightMeters()).minus(ElevatorConstants.kMinHeight);
        Distance maxElevatorTravel = ElevatorConstants.kMaxHeight.minus(ElevatorConstants.kMinHeight);
        double elevatorPercentTravel = currentElevatorTravel.in(Meters) / maxElevatorTravel.in(Meters);

        double speedDifference = OCXboxController.kSpeedDefault - OCXboxController.kSpeedSlow;

        controller.setDriveSpeed(OCXboxController.kSpeedDefault - (elevatorPercentTravel * speedDifference));

    }

    public void tipProtection(){
        double rollRadians = Math.abs(drive.getRotation3d().getX());
        double pitchRadians = Math.abs(drive.getRotation3d().getY());
        double angleTolerance = ElevatorConstants.kTipAngleTolerance.in(Radians);

        if ((rollRadians >= angleTolerance) || (pitchRadians >= angleTolerance)){
            elevator.setHeightC(ElevatorConstants.kMinHeight);
        }
    }
}
