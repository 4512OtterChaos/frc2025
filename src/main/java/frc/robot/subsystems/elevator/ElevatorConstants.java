package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorConstants {
    public static final int kLeftMotorID = 9;
    public static final int kRightMotorID = 10;

    public static final int kGearRatio = 60; //TODO: Use correct gear ratio

    public static final double kElevatorMaxHeight = 72; //As measured from the top of the cariage TODO: Use real val
    public static final double kElevatorMinHeight = 12; //TODO: Use real val

    public static final double kStallThresholdAmps = 20;
    public static final double kStallThresholdSeconds = 0.25;

    // (applied to left motor, right motor follows)
    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = kGearRatio;

        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40;

        // SoftwareLimitSwitchConfigs limits = kConfig.SoftwareLimitSwitch;//TODO: Software limit switches???
        // limits.ForwardSoftLimitEnable = true;
        // limits.ForwardSoftLimitThreshold = kMaxAngle.getRotations();
        // limits.ReverseSoftLimitEnable = true;
        // limits.ReverseSoftLimitThreshold = kHomeAngle.getRotations();

        Slot0Configs control = kConfig.Slot0; //TODO: Update PID
        control.kP = 30;
        control.kI = 0;
        control.kD = 0;

        control.GravityType = GravityTypeValue.Elevator_Static; //TODO: Update Elevator k constants
        control.kG = 0.05;
        control.kS = 0.1;
        control.kV = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic; //TODO: Use correct vals
        mm.MotionMagicCruiseVelocity = 2; // rotations per second
        mm.MotionMagicAcceleration = 5; // rotations per second per second
    }
}
