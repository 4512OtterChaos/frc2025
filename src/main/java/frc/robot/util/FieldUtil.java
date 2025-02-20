package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldUtil {
    public static final Translation2d kSpeakerTrl = new Translation2d(
        0,
        5.5
    );
    // parallel distance from the wall to the front of subwoofer
    public static final double kWallToSubwooferFrontDist = Units.inchesToMeters(36.125);

    // Bottom of the speaker from the ground
    public static final double kSpeakerBottomHeight = Units.inchesToMeters(78);
    // Top of the speaker from the ground
    public static final double kSpeakerTopHeight = Units.inchesToMeters(82.875);
    // Width of the speaker opening
    public static final double kSpeakerWidth = Units.inchesToMeters(41.375);
    // Extension of the speaker opening out in to the feild
    public static final double kSpeakerExtension = Units.inchesToMeters(18);

    
    // Speaker angle above ground parallel
    public static final Rotation2d kSpeakerAngle = Rotation2d.fromDegrees(14);

    public static Rotation2d getAngleToSpeaker(Translation2d from){
        return kSpeakerTrl.minus(from).getAngle();
    }
}
