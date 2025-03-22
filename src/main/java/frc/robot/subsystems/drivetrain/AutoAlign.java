package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class AutoAlign {
    public static record Threshold(
            Distance linearPosTol, LinearVelocity linearVelTol,
            Angle angularPosTol, AngularVelocity angularVelTol
        ) {
    }

    public SwerveDriveLimiter standardLimiter = kAlignLimiter.copy();

    public Threshold finalAlignThreshold = new Threshold(
        Meters.of(kFinalAlignLinearPosTol),
        MetersPerSecond.of(kFinalAlignLinearVelTol),
        Radians.of(kFinalAlignAngularPosTol),
        RadiansPerSecond.of(kFinalAlignDistAngularVelTol)
    );
    public SwerveDriveLimiter slowLimiter = kAlignSlowLimiter.copy();

    public Threshold completeThreshold = new Threshold(
        Meters.of(kPathDrivePosTol),
        MetersPerSecond.of(kPathDriveVelTol),
        Radians.of(kPathTurnPosTol),
        RadiansPerSecond.of(kPathTurnVelTol)
    );

    public AutoAlign(String name) {
    }

    public ChassisSpeeds calculateTargetSpeeds(Pose2d currentPose, Pose2d goalPose) {
        var relative = currentPose.relativeTo(goalPose);

        // Calculate PID output
        var targetSpeeds = new ChassisSpeeds();
        double pidX = pathXController.calculate(
            currentPose.getX(), goalPose.getX()
        );
        double pidY = pathYController.calculate(
            currentPose.getY(), goalPose.getY()
        );
        double pidHypot = Math.hypot(pidX, pidY);
        double pidScale = 1;
        if (pidHypot > 1e-5) {
            pidScale = Math.min(standardLimiter.linearTopSpeed.in(MetersPerSecond), pidHypot) / pidHypot; // Clamp pid output to drivespeed
        }
        targetSpeeds.vxMetersPerSecond = pidX * pidScale;
        targetSpeeds.vyMetersPerSecond = pidY * pidScale;
        targetSpeeds.omegaRadiansPerSecond = pathThetaController.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians()
        );

        // if very close, avoid small outputs
        if (relative.getTranslation().getNorm() < kStopAlignTrlDist) {
            targetSpeeds.vxMetersPerSecond = 0;
            targetSpeeds.vyMetersPerSecond = 0;
        }
        if (Math.abs(relative.getRotation().getRadians()) < kStopAlignRotDist) {
            targetSpeeds.omegaRadiansPerSecond = 0;
        }

        return targetSpeeds;
    }
}
