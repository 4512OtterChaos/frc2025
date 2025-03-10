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

    public static final int kGearRatio = 9;
    public static final Distance kSprocketPD = Inches.of(1.76);

    public static final Mass kCarriageMass = Pounds.of(15);
    
    // public static final Distance kMinHeight = Inches.of(17.75); // As measured from the top of the cariage 
    // public static final Distance kMaxHeight = Inches.of(kMinHeight.in(Inches) + 58); // 58in of travel

    // public static final Distance kL1Height = Inches.of(kMinHeight.in(Inches) + 8);
    // public static final Distance kL2Height = Inches.of(kMinHeight.in(Inches) + 12);
    // public static final Distance kL3Height = Inches.of(kMinHeight.in(Inches) + 28);
    // public static final Distance kL4Height = kMaxHeight;

    public static final Distance kMinHeight = Meters.of(0); // As measured from the top of the cariage 
    public static final Distance kMaxHeight = Meters.of(48); // 58in of travel

    public static final Distance kL1Height = Meters.of(5);
    public static final Distance kL2Height = Meters.of(13);
    public static final Distance kL3Height = Meters.of(26);
    public static final Distance kL4Height = Meters.of(47.75);
    
    public static final Distance kHeightTolerance = Meters.of(0.5);
    public static final Angle kTipAngleTolerance = Degrees.of(10);

    public static final double kStallThresholdAmps = 20;
    public static final double kStallThresholdSeconds = 0.25;

    // (applied to left motor, right motor follows)
    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        // feedback.SensorToMechanismRatio = kGearRatio        *   (Math.PI * sprocketPitchDiameter.in(Meters))  *   2;
        //          motor rotations  -->  shaft rotations  -->  chain travel                                 -->  carriage travel

        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40;

        SoftwareLimitSwitchConfigs limits = kConfig.SoftwareLimitSwitch;
        limits.ForwardSoftLimitEnable = false;
        limits.ForwardSoftLimitThreshold = kMaxHeight.in(Meters);
        limits.ReverseSoftLimitEnable = false;
        limits.ReverseSoftLimitThreshold = kMinHeight.in(Meters);

        Slot0Configs control = kConfig.Slot0;
        control.kP = 5;
        control.kI = 0;
        control.kD = 0;

        control.GravityType = GravityTypeValue.Elevator_Static;
        control.kG = 0.2;
        control.kS = 0.1;
        control.kV = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = 100; // meters per second
        mm.MotionMagicAcceleration = 250; // meters per second per second
    }

    public static Distance motorAngleToElevDist(Angle angle) {
        return Meters.of(angle.in(Rotations) / kGearRatio * (kSprocketPD.in(Meters) * Math.PI));
    }

    public static Angle elevDistToMotorAngle(Distance dist) {
        return Rotations.of(dist.in(Meters) / (kSprocketPD.in(Meters) * Math.PI) * kGearRatio);
    }

    public static final ElevatorSim model = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        kGearRatio,
        kCarriageMass.in(Kilograms),
        kSprocketPD.in(Meters) / 2.0,
        0,
        kMaxHeight.in(Meters),
        true,
        0
    );
}
