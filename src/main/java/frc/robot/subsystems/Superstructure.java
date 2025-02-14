package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;


public class Superstructure {
    private CommandSwerveDrivetrain drive;
    private Manipulator manipulator;
    private Elevator elevator;

    public Superstructure(CommandSwerveDrivetrain drive, Manipulator manipulator, Elevator elevator) {
        this.drive = drive;
        this.manipulator = manipulator;
        this.elevator = elevator;
    }

    public Command intake(){
        return sequence(
            elevator.setHeightC(ElevatorConstants.kMinHeight).withTimeout(0.75).unless(()->elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.in(Inches)),
            manipulator.setVoltageInC()
        );
    }

    public Command outtake(){
        return sequence(
            elevator.setHeightC(ElevatorConstants.kMinHeight).withTimeout(0.75).unless(()->elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.in(Inches)),
            manipulator.setVoltageOutC()
        );
    }

    public Command chooseScorePoint(){//TODO:Just like make it work/do it
        return sequence(
            elevator.setHeightC(ElevatorConstants.kMinHeight).withTimeout(0.75).unless(()->elevator.getElevatorHeightMeters() <= ElevatorConstants.kMinHeight.in(Inches)),
            manipulator.setVoltageOutC()
        );
    }
}
