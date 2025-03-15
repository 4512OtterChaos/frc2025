package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorConstants {
    public static final int kLeftMotorID = 21;
    public static final int kRightMotorID = 22;

    public static final int kGearRatio = 9; // Motor rotations to sprocket rotations (Carriage is effectively a 1:2 upduction!)
    public static final Distance kSprocketPD = Inches.of(1.76);

    public static final Mass kStage1Mass = Pounds.of(6);
    public static final Mass kCarriageMass = Pounds.of(15);
    
    public static final Distance kMaxTravel = Inches.of(58.95); // Travel of the CARRIAGE

    public static final Distance kL1Height = Inches.of(6.15);
    public static final Distance kL2Height = Inches.of(16);
    public static final Distance kL3Height = Inches.of(32);
    public static final Distance kL4Height = Inches.of(58.6);

    public static final Distance kAlgaeL3Height = Inches.of(26.15);


    public static final Distance kHeightTolerance = Inches.of(0.3);

    public static final Angle kTipAngleTolerance = Degrees.of(10);

    public static final double kStallThresholdAmps = 20;
    public static final double kStallThresholdSeconds = 0.25;

    // (applied to left motor, right motor follows)
    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = 1.0 / motorAngleToCarriageDist(Rotations.of(1)).in(Inches); // 1 motor rotation -> x Inches travelled

        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40;

        SoftwareLimitSwitchConfigs limits = kConfig.SoftwareLimitSwitch;
        // limits.ForwardSoftLimitEnable = false;
        // limits.ForwardSoftLimitThreshold = kMaxTravel.in(Meters);
        // limits.ReverseSoftLimitEnable = false;
        // limits.ReverseSoftLimitThreshold = kMinHeight.in(Meters);

        Slot0Configs control = kConfig.Slot0;
        control.kP = 4;
        control.kI = 0;
        control.kD = 0;

        control.GravityType = GravityTypeValue.Elevator_Static;
        control.kG = 0.2;
        control.kS = 0.1;
        control.kV = 0;
        control.kA = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = Inches.of(120).in(Inches); // inches per second
        mm.MotionMagicAcceleration = Inches.of(300).in(Inches); // inches per second per second
    }

    /** 1 motor rotation to [1.22871 inches, 0.03121 meters] */
    public static Distance motorAngleToCarriageDist(Angle angle) {
        // motor rotations -> shaft rotations -> 1st stage linear travel -> carriage linear travel
        return Meters.of(angle.in(Rotations) / kGearRatio * (kSprocketPD.in(Meters) * Math.PI) * 2);
    }

    public static Angle carriageDistToMotorAngle(Distance dist) {
        return Rotations.of(dist.in(Meters) / 2.0 / (kSprocketPD.in(Meters) * Math.PI) * kGearRatio);
    }

    public static final ElevatorSim model = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        kGearRatio / 2.0, // Carriage upduction
        kCarriageMass.plus(kStage1Mass.div(2)).in(Kilograms),
        kSprocketPD.in(Meters) / 2.0,
        0,
        kMaxTravel.in(Meters),
        true,
        0
    );
}
