package frc.robot.subsystems.funnel;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import static frc.robot.subsystems.funnel.FunnelConstants.*;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.*;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunableNumber;

public class Funnel extends SubsystemBase{
    private TalonFX motor = new TalonFX(kMotorID);
    
    // private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kConfig.Slot0.kS, kConfig.Slot0.kV);
    // private PIDController PID = new PIDController(kConfig.Slot0.kP, kConfig.Slot0.kI, kConfig.Slot0.kD);

    private PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);
    // private VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);


    boolean isManual = true;
    
    double lastFreeTime = Timer.getFPGATimestamp();
    double lastSetTime = Timer.getFPGATimestamp();

    ArrayList<Double> velocitiesRotations = new ArrayList<Double>();
    double sumVelocities = 0;
    double avgVelRotations = 0;

    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    // Tunable numbers
    private final TunableNumber feedSlowVoltage = new TunableNumber("Coral/feedSlowVoltage", kFeedSlowVolts);
    private final TunableNumber feedFastVoltage = new TunableNumber("Coral/feedFastVoltage", kFeedFastVolts);

    // private final TunableNumber rpmPerVolt = new TunableNumber("Funnel/rpmPerVolt", kRPMPerVolt);

    private final TunableNumber coralSensePercentDrop = new TunableNumber("Funnel/coralSensePercentDrop", kCoralSensePercentDrop);
    private final TunableNumber coralDelayedSenseTime = new TunableNumber("Funnel/coralDelayedSenseTime", kCoralDelayedSenseTime);

    private final TunableNumber kP = new TunableNumber("Funnel/kP", kConfig.Slot0.kP);
    private final TunableNumber kD = new TunableNumber("Funnel/kD", kConfig.Slot0.kD);

    private final TunableNumber kS = new TunableNumber("Funnel/kS", kConfig.Slot0.kS);
    private final TunableNumber kV = new TunableNumber("Funnel/kV", kConfig.Slot0.kV);
    
    
    public Funnel(){
        // try applying motor configs
        boolean success = PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(kConfig));
        if (!success) DriverStation.reportWarning("Failed applying Funnel motor configuration!", false);

        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(motor);

        SmartDashboard.putData("Funnel/Subsystem", this);
    }

    @Override
    public void periodic() {
        // if(!isManual) {
        //     motor.setVoltage(ff.calculate(PID.getSetpoint())+PID.calculate(getVelocity().in(RotationsPerSecond)));
        // }

        if (getCurrent() <= kStallCurrent){
            lastFreeTime = Timer.getFPGATimestamp();
        }

        updateAverageVelocity();

        changeTunable();

        log();
    }

    public void setVoltage(double voltage){
        isManual = true;
        lastSetTime = Timer.getFPGATimestamp();
        motor.setVoltage(-voltage);
    }

    public void setTargetPos(Angle position) {
        isManual = false;
        lastSetTime = Timer.getFPGATimestamp();
        motor.setControl(positionRequest.withPosition(position));
    }

    // public void setVelocity(AngularVelocity RPM) {
    //     isManual = false;
    //     lastSetTime = Timer.getFPGATimestamp();
    //     motor.setControl(velocityRequest.withVelocity(RPM));
    // }

    public Angle getPosition() {
        return positionStatus.getValue();
    }

    public AngularVelocity getVelocity(){
        return velocityStatus.getValue();
    }

    public double getCurrent(){
        return statorStatus.getValueAsDouble();
    }

    public void updateAverageVelocity(){
        double currentVel = getVelocity().in(RotationsPerSecond);
        double firstStoredVel = 0;
        if (currentVel > 0.15) {
            velocitiesRotations.add(currentVel);
            if (velocitiesRotations.size() > 100 ){ //TODO: Config numVelocitesStored
                firstStoredVel = velocitiesRotations.get(0);
                velocitiesRotations.remove(0);
            }
            sumVelocities += (currentVel - firstStoredVel);
            avgVelRotations = sumVelocities / velocitiesRotations.size();
        }
    }

    public Trigger isStalled(){
        return new Trigger(() -> Timer.getFPGATimestamp() >= (lastFreeTime + kStallTime));
    }

    public Trigger isCoralDetected(){ //TODO: Use velocity detection
        return new Trigger(() -> {
            double deltaVel = avgVelRotations - getVelocity().in(RotationsPerSecond);
            boolean velChanging = deltaVel/avgVelRotations > coralSensePercentDrop.get();
            boolean velSet = Timer.getFPGATimestamp() - lastSetTime > coralDelayedSenseTime.get();
            return velChanging && velSet;
        });
    }

    public Command setVoltageC(double voltage){
        return run(()->setVoltage(voltage));
    }

    public Command feedCoralC() {
        return run(()->setVoltage(feedFastVoltage.get())).withName("FeedCoral");
    }

    public Command slowFeedCoralC() {
        return run(()->setVoltage(feedSlowVoltage.get())).withName("D:SlowFeedCoral");
    }

    public Command backfeedCoralC() {
        return run(()->setVoltage(-feedFastVoltage.get())).withName("BackfeedCoral");
    }

    /** Does not end */
    public Command setPositionC(Angle position) {
        return run(() -> setTargetPos(position)).withName("SetPosition");
    }

    // public Command holdPositionC() {
    //     return startEnd(
    //         () -> setTargetPos(getPosition()),
    //         () -> {}
    //     ).withName("HoldPosition");
    // }

    private void changeTunable() {
        feedFastVoltage.poll();
        feedSlowVoltage.poll();

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
        SmartDashboard.putNumber("Funnel/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Funnel/Motor Current", getCurrent());
        SmartDashboard.putNumber("Funnel/Rotations", getPosition().in(Rotations));
        SmartDashboard.putNumber("Funnel/Rotations per second", getVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Funnel/Average Rotations per second", avgVelRotations);
        SmartDashboard.putBoolean("Funnel/isStalled", isStalled().getAsBoolean());
        SmartDashboard.putBoolean("Funnel/Coral Detected", isCoralDetected().getAsBoolean());

        SmartDashboard.putNumber("Funnel/Coral Travelled Inches", kFunnelRollerDia.times(Math.PI).per(Rotation).timesDivisor(getPosition()).in(Inches));
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
