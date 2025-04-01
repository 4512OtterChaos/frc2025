package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
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
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunableNumber;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

public class Manipulator extends SubsystemBase {
    private TalonFX motor = new TalonFX(kMotorID);

    private LaserCan sensor = new LaserCan(kSensorID);
    
    // private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kConfig.Slot0.kS, kConfig.Slot0.kV);
    // private PIDController PID = new PIDController(kConfig.Slot0.kP, kConfig.Slot0.kI, kConfig.Slot0.kD);

    private PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);

    boolean isManual = true;

    boolean hasAlgae = false;
    boolean hasCoral = false; //TODO: set true on auto init for certain autos
    
    double lastFreeTime = Timer.getFPGATimestamp();

    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    // Tunable numbers
    private final TunableNumber feedSlowVoltage = new TunableNumber("Coral/feedSlowVoltage", kFeedSlowVolts);
    private final TunableNumber feedFastVoltage = new TunableNumber("Coral/feedFastVoltage", kFeedFastVolts);
    private final TunableNumber scoreCoralVolts = new TunableNumber("Coral/scoreCoralVolts", kScoreCoralVolts);
    private final TunableNumber holdAlgaeVolts = new TunableNumber("Coral/holdAlgaeVolts", kHoldAlgaeVolts);
    private final TunableNumber scoreAlgaeVolts = new TunableNumber("Coral/scoreAlgaeVolts", kScoreAlgaeVolts);

    // private final TunableNumber rpmPerVolt = new TunableNumber("Coral/rpmPerVolt", kRPMPerVolt);


    private final TunableNumber kP = new TunableNumber("Coral/kP", kConfig.Slot0.kP);
    private final TunableNumber kD = new TunableNumber("Coral/kD", kConfig.Slot0.kD);

    private final TunableNumber kS = new TunableNumber("Coral/kS", kConfig.Slot0.kS);
    private final TunableNumber kV = new TunableNumber("Coral/kV", kConfig.Slot0.kV);
    
    
    public Manipulator(){
        // try applying motor configs
        boolean success = PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(kConfig));
        if (!success) DriverStation.reportWarning("Failed applying Manipulator motor configuration!", false);

        try {
            sensor.setRangingMode(kRangingMode);
            sensor.setRegionOfInterest(kRegionOfInterest);
            sensor.setTimingBudget(kTimingBudget);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportWarning("Failed applying LaserCAN configuration: " + e, false);
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
        // if(!isManual) {
        //     motor.setVoltage(ff.calculate(PID.getSetpoint())+PID.calculate(getVelocity().in(RotationsPerSecond)));
        // }

        if (getCurrent() <= kStallCurrent){
            lastFreeTime = Timer.getFPGATimestamp();
        }

        updateGamePeiceState();

        changeTunable();

        log();
    }

    public void setVoltage(double voltage){
        isManual = true;
        motor.setVoltage(-voltage);
    }

    public void setTargetPos(Angle position) {
        isManual = false;
        motor.setControl(positionRequest.withPosition(position));
    }

    // public void setVelocity(double RPM) {
    //     isManual = false;
    //     PID.setSetpoint(RPM);
    // }

    public Angle getPosition() {
        return positionStatus.getValue();
    }

    public AngularVelocity getVelocity(){
        return velocityStatus.getValue();
    }

    public Voltage getVoltage(){
        return voltageStatus.getValue();
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

    public void updateGamePeiceState(){
        String commandName = this.getCurrentCommand().getName();

        if (commandName.equals("ScoreCoral") && isStalled().getAsBoolean()) {
            hasCoral = true;            
        }

        if (commandName.equals("ScoreAlgae")){
            hasAlgae = false;
        }

        if (commandName.equals("ScoreCoral")){
            hasCoral = false;
        }
    }

    public Trigger isCoralDetected(){
        return new Trigger(() -> {
            var measurement = sensor.getMeasurement();
            if (measurement == null) return false;
            boolean withinRange = measurement.distance_mm <= kSensorMaxCoralDist.in(Millimeters);
            boolean validRange = measurement.distance_mm > 0;
            boolean statusOk = measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
            return withinRange && validRange && statusOk;
        });
    }

    public Trigger hasCoral(){
        return new Trigger(() -> hasCoral);
    }

    public Trigger hasAlgae(){
        return new Trigger(() -> hasAlgae);
    }

    public Command setVoltageC(double voltage){
        return run(()->setVoltage(voltage));
    }

    public Command feedCoralSlowC() {
        return run(()->setVoltage(feedSlowVoltage.get())).withName("FeedCoralSlow");
    }

    public Command feedCoralFastC() {
        return run(()->setVoltage(feedFastVoltage.get())).withName("FeedCoralFast");
    }

    public Command backfeedCoralSlowC() {
        return run(()->setVoltage(-feedSlowVoltage.get())).withName("BackfeedCoralSlow");
    }

    public Command scoreCoralC() {
        return run(()->setVoltage(scoreCoralVolts.get())).withName("ScoreCoral");
    }

    public Command holdAlgaeC() {
        return run(()->setVoltage(holdAlgaeVolts.get())).withName("HoldAlgae");
    }

    public Command scoreAlgaeC() {
        return run(()->setVoltage(scoreAlgaeVolts.get())).withName("ScoreAlgae");
    }

    public Command holdCoralVoltsC(){
        return sequence(
            setVoltageC(0).withTimeout(0.5),
            setVoltageC(-0.5).withTimeout(0.5)
            ).repeatedly().withName("D:HoldCoral");
    }

    public Command feedCoralSequenceC() {
        return sequence( // Grab coral quickly, then slowly home it to a consistent position
            feedCoralFastC().until(isCoralDetected().negate()),
            backfeedCoralSlowC().until(isCoralDetected()),
            feedCoralSlowC().until(isCoralDetected().negate()),
            runOnce(() -> hasCoral = true)
        ).withName("FeedCoral");
    }

    public Command feedCoralFastSequenceC() {
        return sequence(
            feedCoralFastC().until(isCoralDetected()),
            feedCoralSequenceC()
        );
    }

    /** Does not end */
    public Command setPositionC(Angle position) {
        return run(() -> setTargetPos(position)).withName("SetPosition");
    }

    public Command defaultCommand() {
        if (hasAlgae){
            return holdAlgaeC().withName("D:HoldAlgae");
        }
        return holdPositionC().withName("D:HoldPosition");
    }

    public Command holdPositionC() {
        return startEnd(
            () -> setTargetPos(getPosition()),
            () -> {}
        ).withName("HoldPosition");
    }

    private void changeTunable() {
        feedSlowVoltage.poll();
        feedFastVoltage.poll();
        scoreCoralVolts.poll();
        holdAlgaeVolts.poll();
        scoreAlgaeVolts.poll();

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
        SmartDashboard.putNumber("Coral/Motor Current", getCurrent());
        SmartDashboard.putNumber("Coral/Rotations", getPosition().in(Rotations));
        SmartDashboard.putNumber("Coral/Rotations per second", getVelocity().in(RotationsPerSecond));
        SmartDashboard.putBoolean("Coral/isStalled", isStalled().getAsBoolean());
        SmartDashboard.putBoolean("Coral/hasCoral", hasCoral);
        SmartDashboard.putBoolean("Coral/hasAlgae", hasAlgae);

        SmartDashboard.putNumber("Coral/Coral Travelled Inches", kCoralRollerDia.times(Math.PI).per(Rotation).timesDivisor(getPosition()).in(Inches));
    }
    

    //########## Simulation

    @Override
    public void simulationPeriodic() {
        var motorSim = motor.getSimState();
        motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        model.setInput(-motorSim.getMotorVoltage());
        model.update(0.02);

        motorSim.setRawRotorPosition(model.getAngularPosition().times(kGearRatio));
        motorSim.setRotorVelocity(model.getAngularVelocity().times(kGearRatio));
        motorSim.setRotorAcceleration(model.getAngularAcceleration().times(kGearRatio));
    }
}
