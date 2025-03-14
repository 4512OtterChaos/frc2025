package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
    
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kConfig.Slot0.kS, kConfig.Slot0.kV);
    private PIDController PID = new PIDController(kConfig.Slot0.kP, kConfig.Slot0.kI, kConfig.Slot0.kD);

    boolean isManual = true;
    
    double lastFreeTime = Timer.getFPGATimestamp();

    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    // Tunable numbers
    private final TunableNumber intakeVoltage = new TunableNumber("Coral/intakeVoltage", kIntakeVoltage);
    private final TunableNumber feedVoltage = new TunableNumber("Coral/feedVoltage", kFeedVoltage);
    private final TunableNumber scoreVoltage = new TunableNumber("Coral/scoreVoltage", kScoreVoltage);
    private final TunableNumber algeaShootVoltage = new TunableNumber("Coral/algeaShootVoltage", kAlgaeShootVoltage);
    private final TunableNumber rpmPerVolt = new TunableNumber("Coral/rpmPerVolt", kRPMPerVolt);


    private final TunableNumber kP = new TunableNumber("Coral/kP", kConfig.Slot0.kP);
    private final TunableNumber kD = new TunableNumber("Coral/kD", kConfig.Slot0.kD);

    private final TunableNumber kS = new TunableNumber("Coral/kS", kConfig.Slot0.kS);
    private final TunableNumber kV = new TunableNumber("Coral/kV", kConfig.Slot0.kV);
    
    
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

        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(motor);

        SmartDashboard.putData("Coral/Subsystem", this);
    }

    @Override
    public void periodic() {
        if(!isManual) {
            motor.setVoltage(ff.calculate(PID.getSetpoint())+PID.calculate(getVelocity()));
        }

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

    public void setVelocity(double RPM) {
        isManual = false;
        PID.setSetpoint(RPM);
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
        return run(()->setVoltage(intakeVoltage.get())).withName("Intake");
    }

    /**
     * 
     * @return A command that sets an scoring voltage.
     */

    public Command setVoltageScoreC(){
        return run(()->setVoltage(scoreVoltage.get())).withName("Scoring");
    }

    public Command algaeOff(){
        return run(()->setVoltage(-2)).withName("AlgaeOff");
    }

    public Command holdCoralC(){
        return sequence(
            setVoltageC(0).withTimeout(0.5),
            setVoltageC(-0.5).withTimeout(0.5)
            ).repeatedly().withName("D:HoldCoral");
    }

    public Command feedCoralC() {
        // return setVoltageInC().until(isCoralDetected().negate()).withName("FeedCoral");
        return sequence(
            // setVoltageInC().withTimeout(0.5),
            setVoltageC(feedVoltage.get())
        ).until(isCoralDetected().negate()).withName("FeedCoral");
        // return sequence(
        //     setVoltageInC().until(isCoralDetected().negate()),
        //     setVoltageC(-3).until(isCoralDetected()),
        //     setVoltageC(3).until(isCoralDetected().negate()),
        //     setVoltageC(3).withTimeout(0.05)
        // );
    }

    public Command algaeShoot() {
        return run(()->setVoltage(algeaShootVoltage.get())).withName("ShootAlgae");
    }

    public Command setVelocityC(double RPM){
        return run(()->setVelocity(RPM));
    }

    public Command setVelocityInC(){
        return run(()->setVelocity(intakeVoltage.get()*rpmPerVolt.get())).withName("Intake PID");
    }

    public Command setVelocityScoreC(){
        return run(()->setVelocity(scoreVoltage.get()*rpmPerVolt.get())).withName("Score PID");
    }

    public Command setVelocityStop(){
        return run(()->setVelocity(0)).withName("Hold Pose PID");
    }

    public Command algaeOffVelocity(){
        return run(()->setVelocity(-2*rpmPerVolt.get())).withName("AlgaeOffVelocity");
    }

    public Command holdCoralVelocityC(){
        return sequence(
            setVelocityC(0).withTimeout(0.5),
            setVelocityC(-0.5*rpmPerVolt.get()).withTimeout(0.5)
            ).repeatedly().withName("D:HoldCoralVelocity");
    }

    public Command feedCoralVelocityC() {
        return sequence(
            setVelocityC(feedVoltage.get()*rpmPerVolt.get())
        ).until(isCoralDetected().negate()).withName("FeedCoralVelocity");
    }

    public Command algaeShootVelocity() {
        return run(()->setVelocity(algeaShootVoltage.get()*rpmPerVolt.get())).withName("ShootAlgaeVelocity");
    }

    private void changeTunable() {
        intakeVoltage.poll();
        scoreVoltage.poll();

        kP.poll();
        kD.poll();
        kS.poll();
        kV.poll();

        int hash = hashCode();
        // PID
        if (kP.hasChanged(hash) || kD.hasChanged(hash) || kS.hasChanged(hash) || kV.hasChanged(hash)) {
            kConfig.Slot0.kP = kP.get();
            kConfig.Slot0.kD = kD.get();
            kConfig.Slot0.kS = kS.get();
            kConfig.Slot0.kV = kV.get();
            motor.getConfigurator().apply(kConfig.Slot0);
        }
    }

    private void log() {
        BaseStatusSignal.refreshAll(voltageStatus, positionStatus, velocityStatus, statorStatus);
        SmartDashboard.putNumber("Coral/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Coral/Motor Velocity", getVelocity());
        SmartDashboard.putNumber("Coral/Motor Current", getCurrent());
        SmartDashboard.putBoolean("Coral/isStalled", isStalled().getAsBoolean());
    }
    

    //########## Simulation

    @Override
    public void simulationPeriodic() {
        var motorSim = motor.getSimState();
        motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        model.setInput(motorSim.getMotorVoltage());
        model.update(0.02);

        motorSim.setRawRotorPosition(model.getAngularPosition().times(kGearRatio));
        motorSim.setRotorVelocity(model.getAngularVelocity().times(kGearRatio));
        motorSim.setRotorAcceleration(model.getAngularAcceleration().times(kGearRatio));
    }
}
