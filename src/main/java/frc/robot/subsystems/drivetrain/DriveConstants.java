package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

public class DriveConstants {
    // Meters, radians

    public static final Distance kFrameWidth = Inches.of(28);
    public static final Distance kFrameLength = Inches.of(28);
    public static final Distance kRobotWidth = kFrameWidth.plus(Inches.of(3.25).times(2));
    public static final Distance kRobotLength = kFrameLength.plus(Inches.of(3.25).times(2));

    public static final double kMaxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double kMaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    // Normal driving speed at 100% controller input
    public static final double kDriveSpeed = 0.75 * kMaxLinearSpeed;
    public static final double kTurnSpeed = 0.35 * kMaxAngularRate;

    // Normal driving acceleration limits
    public static final double kLinearAccel = FeetPerSecondPerSecond.of(30).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kLinearDecel = FeetPerSecondPerSecond.of(40).in(MetersPerSecondPerSecond);
    public static final double kAngularAccel = RotationsPerSecondPerSecond.of(6).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecel = RotationsPerSecondPerSecond.of(10).in(RadiansPerSecondPerSecond);

    public static final SwerveDriveLimiter kStandardLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(kDriveSpeed),
        MetersPerSecondPerSecond.of(kLinearAccel),
        MetersPerSecondPerSecond.of(kLinearDecel),
        RadiansPerSecond.of(kTurnSpeed),
        RadiansPerSecondPerSecond.of(kAngularAccel),
        RadiansPerSecondPerSecond.of(kAngularDecel)
    );
    
    // Driving speed when elevator fully extended
    public static final double kDriveSpeedTippy = 0.25 * kMaxLinearSpeed;
    public static final double kTurnSpeedTippy = 0.2 * kMaxAngularRate;

    // Driving acceleration limits when elevator fully extended
    public static final double kLinearAccelTippy = FeetPerSecondPerSecond.of(12).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kLinearDecelTippy = FeetPerSecondPerSecond.of(12).in(MetersPerSecondPerSecond);
    public static final double kAngularAccelTippy = RotationsPerSecondPerSecond.of(4).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecelTippy = RotationsPerSecondPerSecond.of(4).in(RadiansPerSecondPerSecond);

    public static final SwerveDriveLimiter kTippyLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(kDriveSpeedTippy),
        MetersPerSecondPerSecond.of(kLinearAccelTippy),
        MetersPerSecondPerSecond.of(kLinearDecelTippy),
        RadiansPerSecond.of(kTurnSpeedTippy),
        RadiansPerSecondPerSecond.of(kAngularAccelTippy),
        RadiansPerSecondPerSecond.of(kAngularDecelTippy)
    );

    // Path following constants
    public static final double kPathDriveKP = 5;
    public static final double kPathDriveKI = 0;
    public static final double kPathDriveKD = 0;
    public static final double kPathDrivePosTol = Inches.of(1).in(Meters);
    public static final double kPathDriveVelTol = Inches.of(3).in(Meters);
    public static final double kPathTurnKP = 8;
    public static final double kPathTurnKI = 0;
    public static final double kPathTurnKD = 0.1;
    public static final double kPathTurnPosTol = Degrees.of(4).in(Radians);
    public static final double kPathTurnVelTol = Degrees.of(8).in(Radians);

    // Driving speed for auto-alignment
    public static final double kDriveSpeedAlign = 0.6 * kMaxLinearSpeed;
    public static final double kTurnSpeedAlign = 0.5 * kMaxAngularRate;
    // Driving acceleration for auto-alignment
    public static final double kLinearAccelAlign = FeetPerSecondPerSecond.of(30).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kAngularAccelAlign = RotationsPerSecondPerSecond.of(6).in(RadiansPerSecondPerSecond);

    public static final SwerveDriveLimiter kAlignLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(kDriveSpeedAlign),
        MetersPerSecondPerSecond.of(kLinearAccelAlign),
        MetersPerSecondPerSecond.of(kLinearAccelAlign),
        RadiansPerSecond.of(kTurnSpeedAlign),
        RadiansPerSecondPerSecond.of(kAngularAccelAlign),
        RadiansPerSecondPerSecond.of(kAngularAccelAlign)
    );

    // Threshold to use final alignment speeds
    public static final double kFinalAlignLinearPosTol = Feet.of(1).in(Meters);
    public static final double kFinalAlignLinearVelTol = Feet.of(1).in(Meters);
    public static final double kFinalAlignAngularPosTol = Degrees.of(15).in(Radians);
    public static final double kFinalAlignDistAngularVelTol = Degrees.of(15).in(Radians);

    // Driving speed for final auto-alignment
    public static final double kDriveSpeedAlignSlow = 0.25 * kMaxLinearSpeed;
    public static final double kTurnSpeedAlignSlow = 0.2 * kMaxAngularRate;
    // Driving acceleration for final auto-alignment
    public static final double kLinearAccelAlignSlow = FeetPerSecondPerSecond.of(12).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kAngularAccelAlignSlow = RotationsPerSecondPerSecond.of(4).in(RadiansPerSecondPerSecond);

    public static final SwerveDriveLimiter kAlignSlowLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(kDriveSpeedAlignSlow),
        MetersPerSecondPerSecond.of(kLinearAccelAlignSlow),
        MetersPerSecondPerSecond.of(kLinearAccelAlignSlow),
        RadiansPerSecond.of(kTurnSpeedAlignSlow),
        RadiansPerSecondPerSecond.of(kAngularAccelAlignSlow),
        RadiansPerSecondPerSecond.of(kAngularAccelAlignSlow)
    );

    // Threshold to output zero when close
    public static final double kStopAlignTrlDist = Inches.of(0.65).in(Meters);
    public static final double kStopAlignRotDist = Degrees.of(3).in(Radians);
    
}
