package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;

public class SwerveDriveLimiter {
    public LinearVelocity linearTopSpeed;
    public LinearAcceleration linearAcceleration;
    public LinearAcceleration linearDeceleration;

    public AngularVelocity angularTopSpeed;
    public AngularAcceleration angularAcceleration;
    public AngularAcceleration angularDeceleration;

    public SwerveDriveLimiter(
            LinearVelocity linearTopSpeed, LinearAcceleration linearAcceleration, LinearAcceleration linearDeceleration,
            AngularVelocity angularTopSpeed, AngularAcceleration rotationalAcceleration, AngularAcceleration rotationalDeceleration
            ) {
        this.linearTopSpeed = linearTopSpeed;
        this.linearAcceleration = linearAcceleration;
        this.linearDeceleration = linearDeceleration;
        this.angularTopSpeed = angularTopSpeed;
        this.angularAcceleration = rotationalAcceleration;
        this.angularDeceleration = rotationalDeceleration;
    }

    public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds, ChassisSpeeds currentSpeeds, double dt) {
        double linearAcceleration = this.linearAcceleration.in(MetersPerSecondPerSecond);
        double linearDeceleration = this.linearDeceleration.in(MetersPerSecondPerSecond);
        double angularAcceleration = this.angularAcceleration.in(RadiansPerSecondPerSecond);
        double angularDeceleration = this.angularDeceleration.in(RadiansPerSecondPerSecond);

        double currentXVelocity = currentSpeeds.vxMetersPerSecond;
        double currentYVelocity = currentSpeeds.vyMetersPerSecond;
        double currentAngVelocity = currentSpeeds.omegaRadiansPerSecond;

        double targetLinearVel = Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond);
        double clampedLinearVel = Math.min(
            targetLinearVel,
            linearTopSpeed.in(MetersPerSecond)
        );
        double targetLinearVelRatio = 1;
        if (targetLinearVel > 1e-5) {
            targetLinearVelRatio = clampedLinearVel / targetLinearVel;
        }
        double clampedVX = targetSpeeds.vxMetersPerSecond * targetLinearVelRatio;
        double clampedVY = targetSpeeds.vyMetersPerSecond * targetLinearVelRatio;
        double targetXAccel = (clampedVX - currentXVelocity) / dt;
        double targetYAccel = (clampedVY - currentYVelocity) / dt;

        double clampedAngularVel = MathUtil.clamp(
            targetSpeeds.omegaRadiansPerSecond,
            -angularTopSpeed.in(RadiansPerSecond),
            angularTopSpeed.in(RadiansPerSecond)
        );
        double targetAngAccel = (clampedAngularVel - currentAngVelocity) / dt;

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

    public SwerveDriveLimiter copy() {
        return new SwerveDriveLimiter(
            linearTopSpeed,
            linearAcceleration,
            linearDeceleration,
            angularTopSpeed,
            angularAcceleration,
            angularDeceleration
        );
    }

    public void copyFrom(SwerveDriveLimiter other) {
        linearTopSpeed = other.linearTopSpeed;
        linearAcceleration = other.linearAcceleration;
        linearDeceleration = other.linearDeceleration;
        angularTopSpeed = other.angularTopSpeed;
        angularAcceleration = other.angularAcceleration;
        angularDeceleration = other.angularDeceleration;
    }

    /**
     * Configures this limiter to have the least aggressive combination of the given limiter's values.
     */
    public void setToSlowestOf(SwerveDriveLimiter a, SwerveDriveLimiter b) {
        linearTopSpeed = (LinearVelocity)Measure.min(a.linearTopSpeed, b.linearTopSpeed);
        linearAcceleration = (LinearAcceleration)Measure.min(a.linearAcceleration, b.linearAcceleration);
        linearDeceleration = (LinearAcceleration)Measure.min(a.linearDeceleration, b.linearDeceleration);
        angularTopSpeed = (AngularVelocity)Measure.min(a.angularTopSpeed, b.angularTopSpeed);
        angularAcceleration = (AngularAcceleration)Measure.min(a.angularAcceleration, b.angularAcceleration);
        angularDeceleration = (AngularAcceleration)Measure.min(a.angularDeceleration, b.angularDeceleration);
    }

    /**
     * Configures this limiter to have the most aggressive combination of the given limiter's values.
     */
    public void setToFastestOf(SwerveDriveLimiter a, SwerveDriveLimiter b) {
        linearTopSpeed = (LinearVelocity)Measure.max(a.linearTopSpeed, b.linearTopSpeed);
        linearAcceleration = (LinearAcceleration)Measure.max(a.linearAcceleration, b.linearAcceleration);
        linearDeceleration = (LinearAcceleration)Measure.max(a.linearDeceleration, b.linearDeceleration);
        angularTopSpeed = (AngularVelocity)Measure.max(a.angularTopSpeed, b.angularTopSpeed);
        angularAcceleration = (AngularAcceleration)Measure.max(a.angularAcceleration, b.angularAcceleration);
        angularDeceleration = (AngularAcceleration)Measure.max(a.angularDeceleration, b.angularDeceleration);
    }

    /**
     * Configures this limiter to have the values of the interpolation from limiter a to b by t.
     * @param a "low" bound limiter
     * @param b "high" bound limiter
     * @param t Interpolation ratio, where t == 0 would return limiter a and t == 1 would return limiter b. 
     */
    public void setToLerpOf(SwerveDriveLimiter a, SwerveDriveLimiter b, double t) {
        linearTopSpeed = a.linearTopSpeed.plus(b.linearTopSpeed.minus(a.linearTopSpeed).times(t));
        linearAcceleration = a.linearAcceleration.plus(b.linearAcceleration.minus(a.linearAcceleration).times(t));
        linearDeceleration = a.linearDeceleration.plus(b.linearDeceleration.minus(a.linearDeceleration).times(t));
        angularTopSpeed = a.angularTopSpeed.plus(b.angularTopSpeed.minus(a.angularTopSpeed).times(t));
        angularAcceleration = a.angularAcceleration.plus(b.angularAcceleration.minus(a.angularAcceleration).times(t));
        angularDeceleration = a.angularDeceleration.plus(b.angularDeceleration.minus(a.angularDeceleration).times(t));
    }
}
