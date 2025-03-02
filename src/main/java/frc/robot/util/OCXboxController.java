package frc.robot.util;

import java.time.Period;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Custom {@link CommandXboxController} wrapper to add convenience features for
 * driving.
 */
public class OCXboxController extends CommandXboxController {

    private static final double kDeadband = 0.1;

    public static final TunableNumber kSpeedSlow = new TunableNumber("Controller/speedSlow", .3);
    public static final TunableNumber kSpeedDefault = new TunableNumber("Controller/speedDefault", .55);
    public static final double kSpeedFast = 0.8;
    public static final double kSpeedMax = 1.0;
    
    private double drivespeed = kSpeedDefault.get();
    private double turnSpeed = kTurnSpeed.get();
    public static final double kTurnSpeedSlow = 0.15;
    public static final TunableNumber kTurnSpeed = new TunableNumber("Controller/turnSpeed", .35);;

    /**
     * Constructs XboxController on DS joystick port.
     */
    public OCXboxController(int port) {
        super(port);
    }

    public void setDriveSpeed(double drivespeed) {
        this.drivespeed = drivespeed;
    }

    public double getDriveSpeed() {
        return drivespeed;
    }

    public void setTurnSpeed(double turnSpeed) {
        this.turnSpeed = turnSpeed;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    @Override
    public double getLeftY() {
        return -MathUtil.applyDeadband(super.getLeftY(), kDeadband);
    }

    @Override
    public double getLeftX() {
        return -MathUtil.applyDeadband(super.getLeftX(), kDeadband);
    }

    @Override
    public double getRightY() {
        return -MathUtil.applyDeadband(super.getRightY(), kDeadband);
    }

    @Override
    public double getRightX() {
        return -MathUtil.applyDeadband(super.getRightX(), kDeadband);
    }

    /**
     * Finds target drive speeds after applying deadband and square to controller inputs.
     * @param maxLinearVel Maximum linear drive velocity in meters per second
     * @param maxAngularVel Maximum angular drive velocity in radians per second
     * @return Target robot-relative drive speeds as proportions (-1 to 1).
     */
    public ChassisSpeeds getSpeeds(double maxLinearVel, double maxAngularVel) {
        double forward = getLeftY();
        double strafe = getLeftX();
        double turn = getRightX();

        // We want to square inputs to get better low-speed control
        // We must scale (forward, strafe) by the magnitude of their vector instead to preserve direction
        // Controller stick (x,y) does not actually make a perfect circle, clamp below 1.0
        double mag = Math.min(Math.hypot(forward, strafe), 1);
        forward *= mag;
        strafe *= mag;
        turn *= turn * Math.signum(turn);

        // Convert to real units
        forward *= maxLinearVel * drivespeed;
        strafe *= maxLinearVel * drivespeed;
        turn *= maxAngularVel * turnSpeed;
        return new ChassisSpeeds(forward, strafe, turn);
    }

    /** Rumble both controller sides */
    public void rumble(double value){
        getHID().setRumble(RumbleType.kRightRumble, value);
        getHID().setRumble(RumbleType.kLeftRumble, value);
    }
    /** Rumble one controller side */
    public void rumble(boolean left, double value){
        RumbleType side = left ? RumbleType.kLeftRumble : RumbleType.kRightRumble;
        getHID().setRumble(side, value);
    }

    public static void periodic() {
        changeTunable();
    }

    private static void changeTunable() {
        kSpeedSlow.poll();
        kSpeedDefault.poll();
        kTurnSpeed.poll();
    }
}
