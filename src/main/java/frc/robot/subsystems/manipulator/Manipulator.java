package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.TunableNumber;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

public class Manipulator extends SubsystemBase {
    private TalonFX motor = new TalonFX(kMotorID);

    private LaserCan sensor = new LaserCan(kSensorID);
    //TODO: Use sensor for methods
    
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kConfig.Slot0.kS, kConfig.Slot0.kV);
    private PIDController PID = new PIDController(kConfig.Slot0.kP, kConfig.Slot0.kI, kConfig.Slot0.kD);

    boolean isManual = true;
    
    double lastFreeTime = Timer.getFPGATimestamp();

    private final StatusSignal<Double> dutyStatus = motor.getDutyCycle();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    // Tunable numbers
    private final TunableNumber intakeVoltage = new TunableNumber("Coral/intakeVoltage", kIntakeVoltage);
    private final TunableNumber outtakeVoltage = new TunableNumber("Coral/outtakeVoltage", kScoreVoltage);
    
    public Manipulator(){
        // try applying motor configs
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(kConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) DriverStation.reportWarning("Failed applying Manipulator motor configuration!", false);

        try {
            sensor.setRangingMode(LaserCan.RangingMode.SHORT);
            sensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        dutyStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(motor);

        SmartDashboard.putData("Coral/Subsystem", this);
    }

    @Override
    public void periodic() {
        // if(!isManual) {
        //     motor.setVoltage(ff.calculate(PID.getSetpoint())+PID.calculate(getVelocity()));
        // }

        if (getCurrent() <= kStallCurrent){
            lastFreeTime = Timer.getFPGATimestamp();
        }

        changeTunable();

        log();
    }

    public void setVoltage(double voltage){
        isManual=true;
        motor.setVoltage(-voltage);
    }

    public void setVelocity(double floorRPM) {
        isManual = false;
        PID.setSetpoint(floorRPM);
    }

    public double getVelocity(){
        return velocityStatus.getValueAsDouble();
    }

    public double getCurrent(){
        return statorStatus.getValueAsDouble();
    }

    public Trigger isStalled(){
        return new Trigger(() -> Timer.getFPGATimestamp() >= (lastFreeTime + kStallTime));
    }

    /**
     * @return The distance measured by the laser sensor. If unable to measure, -1 mm.
     */
    public Distance getCoralDist(){
        var measurement = sensor.getMeasurement();
        if (measurement == null) {
            return Millimeters.of(-1);
        }
        return Millimeters.of(measurement.distance_mm);
    }

    public Trigger isCoralDetected(){
        return new Trigger(() -> getCoralDist().in(Millimeters) <= kSensorMaxCoralDist.in(Millimeters) && getCoralDist().gt(Millimeters.zero()));
    }

    public Command setVoltageC(double voltage){
        return run(()->setVoltage(voltage));
    }

    /**
     * 
     * @return A command that sets an intaking voltage.
     */
    public Command setVoltageInC(){
        return run(()->setVoltage(intakeVoltage.get()));
    }

    /**
     * 
     * @return A command that sets an outtaking/placing voltage.
     */

    public Command setVoltageOutC(){
        return run(()->setVoltage(outtakeVoltage.get()));
    }

    // public Command setVelocityC(double RPM){
    //     return run(()->setVelocity(RPM));
    // }

    // public Command setVelocityInC(){
    //     return run(()->setVelocity(30));
    // }

    // public Command setVelocityOutC(){
    //     return run(()->setVelocity(-30));
    // }

    // public Command holdCoralC(){
    //     return run(()->setVelocity(0));
    // }

    public Command feedCoralC() {
        // return setVoltageInC().until(isCoralDetected().negate()).withName("FeedCoral");
        return sequence(
            setVoltageInC().withTimeout(0.5),
            setVoltageC(kFeedVoltage)
        ).until(isCoralDetected().negate()).withName("FeedCoral");
        // return sequence(
        //     setVoltageInC().until(isCoralDetected().negate()),
        //     setVoltageC(-3).until(isCoralDetected()),
        //     setVoltageC(3).until(isCoralDetected().negate()),
        //     setVoltageC(3).withTimeout(0.05)
        // );
    }

    private void changeTunable() {
        intakeVoltage.poll();
        outtakeVoltage.poll();
    }

    private void log() {
        SmartDashboard.putNumber("Coral/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Coral/Motor Current", getCurrent());
        SmartDashboard.putBoolean("Coral/isStalled", isStalled().getAsBoolean());

        SmartDashboard.putNumber("Coral/test", intakeVoltage.get());
    }
    
}
