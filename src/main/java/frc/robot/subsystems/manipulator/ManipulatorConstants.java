package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ManipulatorConstants {
    public static int kMotorID = 5;
    public static int kSensorID = 1;

    public static int kGearRatio = 3;


    public static final int kMotorStallLimit = 40;

    public static final double kMotorStallDetection = 15;
    
    public static double kStallCurrent = 20;
    public static double kStallTime = 0.3;

    public static final double kRampRate = 0.08;

    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = kGearRatio;
        //          motor rotations  -->  shaft rotations

        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40;

        Slot0Configs control = kConfig.Slot0; //TODO: Update PID
        control.kP = 0.005;
        control.kI = 0;
        control.kD = 0;

        control.kG = 0;//TODO: Update manipulator k constants
        control.kS = 0.25;
        control.kV = 0.005;
    }
}
