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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static final int kLeftMotorID = 9;
    public static final int kRightMotorID = 10;

    public static final int kGearRatio = 9;
    public static final Distance sprocketPitchDiameter = Inches.of(1.76);

    
    public static final Distance kMinHeight = Inches.of(17.75); // As measured from the top of the cariage 
    public static final Distance kMaxHeight = Inches.of(kMinHeight.in(Inches) + 58); // 58in of travel

    public static final Distance kL1Height = Inches.of(kMinHeight.in(Inches) + 8); // 8in of travel 
    public static final Distance kL2Height = Inches.of(kMinHeight.in(Inches) + 11.625); // 11 5/8in of travel 
    public static final Distance kL3Height = Inches.of(kMinHeight.in(Inches) + 27.4375); // 27 7/16in of travel 
    
    public static final Distance kHeightTolerance = Inches.of(.2);
    public static final Angle kTipAngleTolerance = Degrees.of(10);

    public static final double kStallThresholdAmps = 20;
    public static final double kStallThresholdSeconds = 0.25;

    // (applied to left motor, right motor follows)
    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = kGearRatio        *   (Math.PI * sprocketPitchDiameter.in(Meters))  *   2;
        //          motor rotations  -->  shaft rotations  -->  chain travel                                 -->  carriage travel

        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40;

        SoftwareLimitSwitchConfigs limits = kConfig.SoftwareLimitSwitch;
        limits.ForwardSoftLimitEnable = true;
        limits.ForwardSoftLimitThreshold = kMaxHeight.in(Meters);
        limits.ReverseSoftLimitEnable = true;
        limits.ReverseSoftLimitThreshold = kMinHeight.in(Meters);

        Slot0Configs control = kConfig.Slot0; //TODO: Update PID
        control.kP = 30;
        control.kI = 0;
        control.kD = 0;

        control.GravityType = GravityTypeValue.Elevator_Static; //TODO: Update Elevator k constants
        control.kG = 0.05;
        control.kS = 0.1;
        control.kV = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic; //TODO: Use correct motion magic vals
        mm.MotionMagicCruiseVelocity = 1.5; // meters per second
        mm.MotionMagicAcceleration = 3.5; // meters per second per second
    }
}
