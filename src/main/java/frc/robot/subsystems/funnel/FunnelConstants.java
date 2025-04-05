package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FunnelConstants {
    public static int kMotorID = 41;

    public static DCMotor kMotor = DCMotor.getKrakenX60(1);

    public static double kFeedFastVolts = 6;
    public static double kFeedSlowVolts = 3;

    public static double kGearRatio = 4;
    public static Distance kFunnelRollerDia = Inches.of(4.5);

    public static final int kMotorStallLimit = 30;

    public static final double kMotorStallDetection = 15;
    
    public static double kStallCurrent = 20;
    public static double kStallTime = 0.3;

    public static AngularVelocity kCoralDropSenseThreshold = RPM.of(400);


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
        current.StatorCurrentLimit = kMotorStallLimit;

        Slot0Configs control = kConfig.Slot0; // Position PID
        control.kP = 20;
        control.kI = 0;
        control.kD = 0;

        control.kS = 0.25;
        control.kV = 1.0 / Units.radiansToRotations(kMotor.withReduction(kGearRatio).KvRadPerSecPerVolt);
    }

    public static final DCMotorSim model = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(1.2 * 1.0 / kMotor.withReduction(kGearRatio).KvRadPerSecPerVolt, 0.002),
        kMotor
    );
}
