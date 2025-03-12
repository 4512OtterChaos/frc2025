package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

public class DriveConstants {
    // Meters, radians

    public static final double kMaxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double kMaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    // Normal driving speed at 100% controller input
    public static final double kDriveSpeed = 0.55 * kMaxLinearSpeed;
    public static final double kTurnSpeed = 0.3 * kMaxAngularRate;

    // Normal driving acceleration limits
    public static final double kLinearAccel = FeetPerSecondPerSecond.of(30).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kLinearDecel = FeetPerSecondPerSecond.of(40).in(MetersPerSecondPerSecond);
    public static final double kAngularAccel = RotationsPerSecondPerSecond.of(6).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecel = RotationsPerSecondPerSecond.of(10).in(RadiansPerSecondPerSecond);
    
    // Driving speed when elevator fully extended
    public static final double kDriveSpeedTippy = 0.25 * kMaxLinearSpeed;
    public static final double kTurnSpeedTippy = 0.2 * kMaxAngularRate;

    // Driving acceleration limits when elevator fully extended
    public static final double kLinearAccelTippy = FeetPerSecondPerSecond.of(12).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kLinearDecelTippy = FeetPerSecondPerSecond.of(12).in(MetersPerSecondPerSecond);
    public static final double kAngularAccelTippy = RotationsPerSecondPerSecond.of(4).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecelTippy = RotationsPerSecondPerSecond.of(4).in(RadiansPerSecondPerSecond);

    // Path following constants
    public static final double kPathDriveKP = 5;
    public static final double kPathDriveKI = 0;
    public static final double kPathDriveKD = 0.5;
    public static final double kPathDrivePosTol = Inches.of(1).in(Meters);
    public static final double kPathDriveVelTol = Inches.of(2).in(Meters);
    public static final double kPathTurnKP = 7;
    public static final double kPathTurnKI = 0;
    public static final double kPathTurnKD = 0.1;
    public static final double kPathTurnPosTol = Degrees.of(4).in(Radians);
    public static final double kPathTurnVelTol = Degrees.of(5).in(Radians);

    public static final Distance kFrameWidth = Inches.of(28);
    public static final Distance kFrameLength = Inches.of(28);
    public static final Distance kRobotWidth = kFrameWidth.plus(Inches.of(3.25).times(2));
    public static final Distance kRobotLength = kFrameLength.plus(Inches.of(3.25).times(2));
}
