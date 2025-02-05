package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
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

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

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
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();

    public Elevator(){
        // try applying motor configs
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = leftMotor.getConfigurator().apply(kConfig);
            rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
            if (status.isOK()) break;
        }
        if (!status.isOK()) DriverStation.reportWarning("Failed applying Elevator motor configuration!", false);

        dutyStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);

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
        if (!isHoming && (currentHeightMeters <= kMinHeight.plus(kHeightTolerance).in(Meters)) && (targetHeight.in(Meters) <= kMinHeight.plus(kHeightTolerance.times(3)).in(Meters))) {
            adjustedVoltage = Math.min(adjustedVoltage, 0);
            if (!isManual) { // go limp at bottom height
                isManual = true;
                adjustedVoltage = 0;
            }
        }

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

        log();
    }

    public double getElevatorHeightMeters() {
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
        ).until(()->isStalled());
    }

    public void log() {
        SmartDashboard.putNumber("Elevator/Elevator Native", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Elevator Inches", Units.metersToInches(getElevatorHeightMeters()));
        SmartDashboard.putNumber("Elevator/Elevator Target Inches", targetHeight.in(Inches));
        SmartDashboard.putNumber("Elevator/Motor Current", getCurrent());
        SmartDashboard.putBoolean("Elevator/isStalled", isStalled());
        SmartDashboard.putNumber("Elevator/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Motor Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Elevator/Motor Velocity", getVelocity());
        SmartDashboard.putBoolean("Elevator/Motor Stalled", isStalled());
        SmartDashboard.putNumber("Elevator/Time", Timer.getFPGATimestamp());
    //     SmartDashboard.putData("Elevator/Mech2d", mech);
    }
}
