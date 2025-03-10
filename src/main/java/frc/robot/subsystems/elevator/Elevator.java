package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private Distance targetHeight = kMinHeight;
    private boolean isManual = false;
    private double targetVoltage = 0;

    private double lastNonStallTime = Timer.getFPGATimestamp();


    private boolean isHoming = false;

    private final StatusSignal<Double> dutyStatus = leftMotor.getDutyCycle();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    // private final StatusSignal<Angle> positionStatus2 = rightMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();

    // Tunable numbers
    private final TunableNumber kP = new TunableNumber("Elevator/kP", kConfig.Slot0.kP);
    private final TunableNumber kD = new TunableNumber("Elevator/kD", kConfig.Slot0.kD);

    private final TunableNumber kG = new TunableNumber("Elevator/kG", kConfig.Slot0.kG);
    private final TunableNumber kS = new TunableNumber("Elevator/kS", kConfig.Slot0.kS);
    private final TunableNumber kV = new TunableNumber("Elevator/kV", kConfig.Slot0.kV);

    private final TunableNumber mmCruise = new TunableNumber("Elevator/mmCruise", kConfig.MotionMagic.MotionMagicCruiseVelocity);
    private final TunableNumber mmAccel = new TunableNumber("Elevator/mmAccel", kConfig.MotionMagic.MotionMagicAcceleration);

    private final TunableNumber heightL1Inches = new TunableNumber("Elevator/heightL1Inches", kL1Height.in(Meters));
    private final TunableNumber heightL2Inches = new TunableNumber("Elevator/heightL2Inches", kL2Height.in(Meters));
    private final TunableNumber heightL3Inches = new TunableNumber("Elevator/heightL3Inches", kL3Height.in(Meters));
    private final TunableNumber heightL4Inches = new TunableNumber("Elevator/heightL4Inches", kL4Height.in(Meters));

    public Elevator(){
        // try applying motor configs
        StatusCode statusL = StatusCode.StatusCodeNotInitialized;
        StatusCode statusR = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 1; i++) {
            statusL = leftMotor.getConfigurator().apply(kConfig);
            statusR = rightMotor.getConfigurator().apply(kConfig);
            rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
            // rightMotor.setControl(new CoastOut());
            if (statusL.isOK() && statusR.isOK()) break;
        }
        if (!statusL.isOK() || !statusR.isOK()) DriverStation.reportWarning("Failed applying Elevator motor configuration!", false);

        dutyStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        // ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);

        SmartDashboard.putData("Elevator/Subsystem", this);

        resetElevatorHeight(kMinHeight.in(Meters)); 
    }

    @Override
    public void periodic() {
        // Height safety
        double currentHeightMeters = getElevatorHeightMeters(); 
        double currentKG = kConfig.Slot0.kG;
        double adjustedVoltage = targetVoltage + currentKG;

        if (currentHeightMeters <= kMinHeight.in(Meters)) {
            adjustedVoltage = Math.max(currentKG, adjustedVoltage);
        }
        if (currentHeightMeters >= kMaxHeight.in(Meters)) {
            adjustedVoltage = Math.min(currentKG, adjustedVoltage);
        }
        // if (!isHoming && (currentHeightMeters <= kMinHeight.plus(kHeightTolerance).in(Meters)) && (targetHeight.in(Meters) <= kMinHeight.plus(kHeightTolerance.times(3)).in(Meters))) {
        //     adjustedVoltage = Math.min(adjustedVoltage, 0);
        //     if (!isManual) { // go limp at bottom height
        //         isManual = true;
        //         adjustedVoltage = 0;
        //     }
        // }

        // Voltage/Position control
        if (!isManual) {
            leftMotor.setControl(mmRequest.withPosition(targetHeight.in(Meters))); 
        }
        else {
            leftMotor.setControl(voltageRequest.withOutput(adjustedVoltage));
        }

        // Stall detection
        if (getCurrent() < kStallThresholdAmps) {
            lastNonStallTime = Timer.getFPGATimestamp();
        }

        // //Update mechanism 2D
        // visualizeState(getArmRotations());
        // visualizeSetpoint(targetAngle.getRotations());

        changeTunable(); // update config if tunable numbers have changed

        log(); // log subsystem stats
    }

    public double  getElevatorHeightMeters() {
        positionStatus.refresh();
        return positionStatus.getValueAsDouble();
    }

    public double getTargetMeters() {
        return targetHeight.in(Meters);
    }

    public boolean isWithinTolerance() {
        double error = targetHeight.in(Meters) - getElevatorHeightMeters();
        return Math.abs(error) < kHeightTolerance.in(Meters);
    }

    public void resetElevatorHeight(double meters){
        leftMotor.setPosition(meters);
    }

    public void setVoltage(double volts){
        isManual = true;
        targetVoltage = volts;
    }

    public void setHeight(Distance targetHeight){
        isManual = false;
        this.targetHeight = Meters.of(MathUtil.clamp(targetHeight.in(Meters), kMinHeight.in(Meters), kMaxHeight.in(Meters)));
    }

    public void decreaseHeight(double heightDecreaseMeters){
        isManual = false;
        this.targetHeight = Meters.of(MathUtil.clamp(targetHeight.in(Meters) - heightDecreaseMeters, kMinHeight.in(Meters), getElevatorHeightMeters())); 
    }

    public void stop(){ 
        setVoltage(0);
    }

    public double getVelocity(){
        return velocityStatus.getValueAsDouble();
    }

    public double getCurrent(){
        return statorStatus.getValueAsDouble();
    }

    public boolean isStalled(){
        return (Timer.getFPGATimestamp() - lastNonStallTime) > kStallThresholdSeconds;
    }

    //---------- Command factories

    /** Sets the elevator voltage and ends immediately. */
    public Command setVoltageC(double volts) {
        return runOnce(()->setVoltage(volts));
    }

    /** Sets the target elevator height and ends when it is within tolerance. */
    public Command setHeightC(Distance targetHeight){
        return run(()->setHeight(targetHeight)).until(this::isWithinTolerance);
    }

    /** Sets the target elevator height and ends when it is within tolerance. */
    public Command setHeightC(Supplier<Distance> targetHeight){
        return run(()->setHeight(targetHeight.get())).until(this::isWithinTolerance);
    }

    /** Sets the target elevator height to the L1 height and ends when it is within tolerance. */
    public Command setMinC(){
        return setHeightC(kMinHeight).withName("Go to base");
    }

    /** Sets the target elevator height to the L1 height and ends when it is within tolerance. */
    public Command setL1C(){
        return setHeightC(() -> Meters.of(heightL1Inches.get())).withName("Go to L1");
    }

    /** Sets the target elevator height to the L2 height and ends when it is within tolerance. */
    public Command setL2C(){
        return setHeightC(() -> Meters.of(heightL2Inches.get())).withName("Go to L2");
    }

    /** Sets the target elevator height to the L3 height and ends when it is within tolerance. */
    public Command setL3C(){
        return setHeightC(() -> Meters.of(heightL3Inches.get())).withName("Go to L3");
    }

    /** Sets the target elevator height to the L4 height and ends when it is within tolerance. */
    public Command setL4C(){
        return setHeightC(() -> Meters.of(heightL4Inches.get())).withName("Go to L4");
    }

    /** Runs the elevator into the base, detecting a current spike and resetting the elevator height. */
    public Command homingSequenceC(){
        if(RobotBase.isSimulation()){ return setHeightC(kMinHeight);}
        return startEnd(
            () -> {
                isHoming = true;
                setVoltage(2); //TODO: Update to correct voltage
            },
            () -> {
                isHoming = false;
                setVoltage(0);
                resetElevatorHeight(kMinHeight.in(Meters));
            }
        ).until(()->isStalled()).withName("Homing");
    }

    private void changeTunable() {
        kP.poll();
        kD.poll();
        kG.poll();
        kS.poll();
        kV.poll();
        mmCruise.poll();
        mmAccel.poll();
        heightL1Inches.poll();
        heightL2Inches.poll();
        heightL3Inches.poll();
        heightL4Inches.poll();

        int hash = hashCode();
        // PID
        if (kP.hasChanged(hash) || kD.hasChanged(hash) || kG.hasChanged(hash) || kS.hasChanged(hash) || kV.hasChanged(hash)) {
            kConfig.Slot0.kP = kP.get();
            kConfig.Slot0.kD = kD.get();
            kConfig.Slot0.kG = kG.get();
            kConfig.Slot0.kS = kS.get();
            kConfig.Slot0.kV = kV.get();
            leftMotor.getConfigurator().apply(kConfig.Slot0);
        }
        // Motion magic
        if (mmCruise.hasChanged(hash) || mmAccel.hasChanged(hash)) {
            kConfig.MotionMagic.MotionMagicCruiseVelocity = mmCruise.get();
            kConfig.MotionMagic.MotionMagicAcceleration = mmAccel.get();
            leftMotor.getConfigurator().apply(kConfig.MotionMagic);
        }
    }

    private void log() {
        // BaseStatusSignal.waitForAll(0.005, voltageStatus, positionStatus);
        BaseStatusSignal.refreshAll(voltageStatus, positionStatus, /*positionStatus2,*/ dutyStatus, velocityStatus, statorStatus);
        SmartDashboard.putNumber("Elevator/Elevator Rotor", leftMotor.getRotorPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Elevator/Elevator Rotor2", rightMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Elevator Native", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Elevator Fake Units", Units.metersToInches(getElevatorHeightMeters()));
        SmartDashboard.putNumber("Elevator/Elevator Inches(?)", getElevatorHeightMeters()*(kGearRatio * (Math.PI * sprocketPitchDiameter.in(Meters)) * 2));
        // SmartDashboard.putNumber("Elevator/Elevator Inches2", Units.metersToInches(positionStatus2.getValueAsDouble()));
        SmartDashboard.putNumber("Elevator/Elevator Target Inches", targetHeight.in(Inches));
        SmartDashboard.putNumber("Elevator/Motor Current", getCurrent());
        SmartDashboard.putBoolean("Elevator/isStalled", isStalled());
        SmartDashboard.putNumber("Elevator/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Motor Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Elevator/Motor Velocity", getVelocity());
    //     SmartDashboard.putData("Elevator/Mech2d", mech);
    }
}
