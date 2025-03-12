package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveAccelLimiter {
    public double linearAcceleration;
    public double linearDeceleration;
    public double angularAcceleration;
    public double angularDeceleration;

    public SwerveDriveAccelLimiter(double linearAcceleration, double linearDeceleration, double rotationalAcceleration, double rotationalDeceleration){
        this.linearAcceleration = linearAcceleration;
        this.linearDeceleration = linearDeceleration;
        this.angularAcceleration = rotationalAcceleration;
        this.angularDeceleration = rotationalDeceleration;
    }

    public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds, ChassisSpeeds currentSpeeds, double dt){
        double currentXVelocity = currentSpeeds.vxMetersPerSecond;
        double currentYVelocity = currentSpeeds.vyMetersPerSecond;
        double currentAngVelocity = currentSpeeds.omegaRadiansPerSecond;
        double targetXAccel = (targetSpeeds.vxMetersPerSecond - currentXVelocity) / dt;
        double targetYAccel = (targetSpeeds.vyMetersPerSecond - currentYVelocity) / dt;
        double targetAngAccel = (targetSpeeds.omegaRadiansPerSecond - currentAngVelocity) / dt;

        Rotation2d velHeading;
        if (Math.hypot(targetXAccel, targetYAccel) > 1e-6) {
            velHeading = new Rotation2d(targetXAccel, targetYAccel);
        }
        else { // Already at target speeds
            velHeading = Rotation2d.kZero;
        }
        double cosVelHeading = Math.abs(Math.cos(velHeading.getRadians()));
        double sinVelHeading = Math.abs(Math.sin(velHeading.getRadians()));

        // Limit X accel
        double low = -linearAcceleration;
        double high = linearAcceleration;
        if (Math.signum(currentXVelocity) > 0){
            low = -linearDeceleration;
        }
        else if ((Math.signum(currentXVelocity) < 0)) {
            high = linearDeceleration;
        } 
        targetXAccel = MathUtil.clamp(targetXAccel, low * cosVelHeading, high * cosVelHeading);

        // Limit Y accel
        low = -linearAcceleration;
        high = linearAcceleration;
        if (Math.signum(currentYVelocity) > 0){
            low = -linearDeceleration;
        }
        else if ((Math.signum(currentYVelocity) < 0)) {
            high = linearDeceleration;
        } 
        targetYAccel = MathUtil.clamp(targetYAccel, low * sinVelHeading, high * sinVelHeading);

        // Limit rotational accel
        low = -angularAcceleration;
        high = angularAcceleration;
        if (Math.signum(currentAngVelocity) > 0){
            low = -angularDeceleration;
        }
        else if ((Math.signum(currentAngVelocity) < 0)) {
            high = angularDeceleration;
        } 
        targetAngAccel = MathUtil.clamp(targetAngAccel, low, high);

        return new ChassisSpeeds(
            currentXVelocity + (targetXAccel * dt),
            currentYVelocity + (targetYAccel * dt),
            currentAngVelocity + (targetAngAccel * dt)
        );
    }
}
