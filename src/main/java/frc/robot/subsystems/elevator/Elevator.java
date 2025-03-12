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
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private Distance goalHeight = Meters.of(0);
    private boolean isManual = false;
    private double targetVoltage = 0;

    private double lastNonStallTime = Timer.getFPGATimestamp();


    private boolean isHoming = false;

    private final StatusSignal<Double> dutyStatus = leftMotor.getDutyCycle();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();

    // Tunable numbers
    private final TunableNumber kP = new TunableNumber("Elevator/kP", kConfig.Slot0.kP);
    private final TunableNumber kD = new TunableNumber("Elevator/kD", kConfig.Slot0.kD);

    private final TunableNumber kG = new TunableNumber("Elevator/kG", kConfig.Slot0.kG);
    private final TunableNumber kS = new TunableNumber("Elevator/kS", kConfig.Slot0.kS);
    private final TunableNumber kV = new TunableNumber("Elevator/kV", kConfig.Slot0.kV);
    private final TunableNumber kA = new TunableNumber("Elevator/kA", kConfig.Slot0.kA);

    private final TunableNumber mmCruise = new TunableNumber("Elevator/mmCruiseInches", kConfig.MotionMagic.MotionMagicCruiseVelocity);
    private final TunableNumber mmAccel = new TunableNumber("Elevator/mmAccelInches", kConfig.MotionMagic.MotionMagicAcceleration);

    private final TunableNumber heightL1Inches = new TunableNumber("Elevator/heightL1Inches", kL1Height.in(Inches));
    private final TunableNumber heightL2Inches = new TunableNumber("Elevator/heightL2Inches", kL2Height.in(Inches));
    private final TunableNumber heightL3Inches = new TunableNumber("Elevator/heightL3Inches", kL3Height.in(Inches));
    private final TunableNumber heightL4Inches = new TunableNumber("Elevator/heightL4Inches", kL4Height.in(Inches));

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

        resetElevatorHeight(Inches.of(0)); 
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(voltageStatus, positionStatus, dutyStatus, velocityStatus, statorStatus);
        changeTunable(); // update config if tunable numbers have changed

        // Height safety
        double currentHeightMeters = getHeight().in(Meters); 
        double currentKG = kConfig.Slot0.kG;
        double adjustedVoltage = targetVoltage + currentKG;

        if (currentHeightMeters <= 0) {
            adjustedVoltage = Math.max(currentKG, adjustedVoltage);
        }
        if (currentHeightMeters >= kMaxTravel.in(Meters)) {
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
            leftMotor.setControl(mmRequest.withPosition(goalHeight.in(Inches))); 
        }
        else {
            leftMotor.setControl(voltageRequest.withOutput(adjustedVoltage));
        }

        // Stall detection
        if (getCurrent() < kStallThresholdAmps) {
            lastNonStallTime = Timer.getFPGATimestamp();
        }


        log(); // log subsystem stats
    }

    public Distance getHeight() {
        return Inches.of(positionStatus.getValueAsDouble());
    }

    public Distance getGoalHeight() {
        return goalHeight;
    }

    public boolean isWithinTolerance() {
        return goalHeight.minus(getHeight()).isNear(Inches.zero(), kHeightTolerance);
    }

    public void resetElevatorHeight(Distance height){
        leftMotor.setPosition(height.in(Inches));
    }

    public void setVoltage(double volts){
        isManual = true;
        targetVoltage = volts;
    }

    public void setHeight(Distance targetHeight){
        isManual = false;
        this.goalHeight = Inches.of(MathUtil.clamp(targetHeight.in(Inches), 0, kMaxTravel.in(Inches)));
    }

    public void stop(){ 
        setVoltage(0);
    }

    public LinearVelocity getVelocity(){
        return InchesPerSecond.of(velocityStatus.getValueAsDouble());
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
        return setHeightC(Meters.of(0)).withName("Go to base");
    }

    /** Sets the target elevator height to the L1 height and ends when it is within tolerance. */
    public Command setL1C(){
        return setHeightC(() -> Inches.of(heightL1Inches.get())).withName("Go to L1");
    }

    /** Sets the target elevator height to the L2 height and ends when it is within tolerance. */
    public Command setL2C(){
        return setHeightC(() -> Inches.of(heightL2Inches.get())).withName("Go to L2");
    }

    /** Sets the target elevator height to the L3 height and ends when it is within tolerance. */
    public Command setL3C(){
        return setHeightC(() -> Inches.of(heightL3Inches.get())).withName("Go to L3");
    }

    /** Sets the target elevator height to the L4 height and ends when it is within tolerance. */
    public Command setL4C(){
        return setHeightC(() -> Inches.of(heightL4Inches.get())).withName("Go to L4");
    }

    /** Runs the elevator into the base, detecting a current spike and resetting the elevator height. */
    public Command homingSequenceC(){
        if(RobotBase.isSimulation()){ return setMinC();}
        return startEnd(
            () -> {
                isHoming = true;
                setVoltage(2); //TODO: Update to correct voltage
            },
            () -> {
                isHoming = false;
                setVoltage(0);
                resetElevatorHeight(Inches.of(0));
            }
        ).until(()->isStalled()).withName("Homing");
    }

    private void changeTunable() {
        kP.poll();
        kD.poll();
        kG.poll();
        kS.poll();
        kV.poll();
        kA.poll();
        mmCruise.poll();
        mmAccel.poll();
        heightL1Inches.poll();
        heightL2Inches.poll();
        heightL3Inches.poll();
        heightL4Inches.poll();

        int hash = hashCode();
        // PID
        if (kP.hasChanged(hash) || kD.hasChanged(hash) || kG.hasChanged(hash) || kS.hasChanged(hash) || kV.hasChanged(hash) || kA.hasChanged(hash)) {
            kConfig.Slot0.kP = kP.get();
            kConfig.Slot0.kD = kD.get();
            kConfig.Slot0.kG = kG.get();
            kConfig.Slot0.kS = kS.get();
            kConfig.Slot0.kV = kV.get();
            kConfig.Slot0.kA = kA.get();
            leftMotor.getConfigurator().apply(kConfig.Slot0);
        }
        // Motion magic
        if (mmCruise.hasChanged(hash) || mmAccel.hasChanged(hash)) {
            kConfig.MotionMagic.MotionMagicCruiseVelocity = mmCruise.get();
            kConfig.MotionMagic.MotionMagicAcceleration = mmAccel.get();
            leftMotor.getConfigurator().apply(kConfig.MotionMagic);
        }
    }

    //########## Logging

    private Mechanism2d mech = new Mechanism2d(1, 2.3);
    private MechanismRoot2d mechRoot = mech.getRoot("elevator", 0.5 + kMech2dOffsetX.in(Meters), kMech2dOffsetZ.in(Meters));
    private MechanismLigament2d mechElev = mechRoot.append(
            new MechanismLigament2d("elevator", 1, 90, 10, new Color8Bit(235, 137, 52)));
    private MechanismLigament2d mechCoral = mechElev.append(
            new MechanismLigament2d("coral", kMech2dCoralLength.in(Meters), kMech2dCoralAngle.in(Degrees), 6, new Color8Bit(240, 240, 220)));

    private void log() {
        SmartDashboard.putNumber("Elevator/Rotor Position", leftMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Position Inches", getHeight().in(Inches));
        SmartDashboard.putNumber("Elevator/Goal Inches", goalHeight.in(Inches));
        SmartDashboard.putNumber("Elevator/Velocity Inches", getVelocity().in(InchesPerSecond));
        SmartDashboard.putNumber("Elevator/Motor Current", getCurrent());
        SmartDashboard.putBoolean("Elevator/isStalled", isStalled());
        SmartDashboard.putNumber("Elevator/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Motor Target Voltage", targetVoltage);
        
        SmartDashboard.putNumber("Elevator/Setpoint Inches", leftMotor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Acceleration Inches", leftMotor.getAcceleration().getValueAsDouble());

        mechElev.setLength(getHeight().plus(kMech2dCoralZ).in(Meters));
        SmartDashboard.putData("Elevator/Mech2d", mech);
    }


    //########## Simulation

    @Override
    public void simulationPeriodic() {
        var leftSim = leftMotor.getSimState();
        leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var rightSim = rightMotor.getSimState();
        rightSim.Orientation = ChassisReference.Clockwise_Positive;
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        model.setInput(leftSim.getMotorVoltage());
        model.update(0.02);

        leftSim.setRawRotorPosition(ElevatorConstants.carriageDistToMotorAngle(Meters.of(model.getPositionMeters())));
        leftSim.setRotorVelocity(ElevatorConstants.carriageDistToMotorAngle(Meters.of(model.getVelocityMetersPerSecond())).per(Second));
        rightSim.setRawRotorPosition(ElevatorConstants.carriageDistToMotorAngle(Meters.of(model.getPositionMeters())));
        rightSim.setRotorVelocity(ElevatorConstants.carriageDistToMotorAngle(Meters.of(model.getVelocityMetersPerSecond())).per(Second));
    }
}
