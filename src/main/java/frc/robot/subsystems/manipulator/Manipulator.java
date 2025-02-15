package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

public class Manipulator extends SubsystemBase{
    private TalonFX motor = new TalonFX(kMotorID);
    //TODO: Add sensor and use sensor for methods
    
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kConfig.Slot0.kS, kConfig.Slot0.kV);
    private PIDController PID = new PIDController(kConfig.Slot0.kP, kConfig.Slot0.kI, kConfig.Slot0.kD);

    boolean isManual = true;
    
    double lastFreeTime = Timer.getFPGATimestamp();

    private final StatusSignal<Double> dutyStatus = motor.getDutyCycle();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();
    
    public Manipulator(){
        // try applying motor configs
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(kConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) DriverStation.reportWarning("Failed applying Manipulator motor configuration!", false);

        dutyStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(motor);

        SmartDashboard.putData("Intake/Subsystem", this);
    }

    // @Override
    public void periodic() {
        if(!isManual) {
            motor.setVoltage(ff.calculate(PID.getSetpoint())+PID.calculate(getVelocity()));
        }

        if (getCurrent() <= kStallCurrent){
            lastFreeTime = Timer.getFPGATimestamp();
        }

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

    public boolean isStalled(){
        return Timer.getFPGATimestamp() >= (lastFreeTime + kStallTime);
    }

    public double getVelocity(){
        return velocityStatus.getValueAsDouble();
    }

    public double getCurrent(){
        return statorStatus.getValueAsDouble();
    }

    public Command setVoltageC(double voltage){
        return run(()->setVoltage(voltage));
    }

    /**
     * 
     * @return A command that sets an intaking voltage.
     */
    public Command setVoltageInC(){
        return run(()->setVoltage(1.75)).until(()->isStalled());
    }

    /**
     * 
     * @return A command that sets an outtaking/placing voltage.
     */

    public Command setVoltageOutC(){
        return run(()->setVoltage(-0.3));
    }

    public Command setVelocityC(double RPM){
        return run(()->setVelocity(RPM));
    }

    public Command setVelocityInC(){
        return run(()->setVelocity(30));
    }

    public Command setVelocityOutC(){
        return run(()->setVelocity(-30));
    }

    public Command holdCoralC(){
        return run(()->setVelocity(0));
    }

    public void log() {
        SmartDashboard.putNumber("Intake/Motor Voltage", voltageStatus.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Motor Current", getCurrent());
        SmartDashboard.putBoolean("Intake/isStalled", isStalled());
    }
    
}
