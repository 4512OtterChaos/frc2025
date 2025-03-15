package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ManipulatorConstants {
    public static int kMotorID = 31;
    public static int kSensorID = 0;

    public static DCMotor kMotor = DCMotor.getKrakenX60(1);

    public static double kFeedFastVolts = 0.9;
    public static double kFeedSlowVolts = 0.5;

    public static double kScoreCoralVolts = 3.5;

    // Hold at x amps torque. ~1.7 volts per Nm, ~50 amps per Nm at stall
    public static double kHoldAlgaeVolts = kMotor.getVoltage(kMotor.getTorque(20), 0);
    public static double kScoreAlgaeVolts = -5;

    public static double kRPMPerVolt = 20; //TODO: Tune to reflect reality, probably use math

    public static int kGearRatio = 3;
    public static Distance kCoralRollerDia = Inches.of(2);
    public static Distance kAlgaeRollerDia = Inches.of(3);

    public static RangingMode kRangingMode = RangingMode.SHORT;
    public static RegionOfInterest kRegionOfInterest = new RegionOfInterest(8, 8, 6, 6);
    public static TimingBudget kTimingBudget = TimingBudget.TIMING_BUDGET_20MS;

    public static final Distance kSensorMaxCoralDist = Inches.of(5);

    public static final int kMotorStallLimit = 40;

    public static final double kMotorStallDetection = 15;
    
    public static double kStallCurrent = 20;
    public static double kStallTime = 0.3;


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

        Slot0Configs control = kConfig.Slot0; // Position PID
        control.kP = 20;
        control.kI = 0;
        control.kD = 0;

        control.kS = 0.25;
        control.kV = 1.0 / Units.radiansToRotations(kMotor.withReduction(kGearRatio).KvRadPerSecPerVolt);
    }

    public static final DCMotorSim model = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(1.3 * 1.0 / kMotor.withReduction(kGearRatio).KvRadPerSecPerVolt, 0.002),
        kMotor
    );
}
