package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String kCameraNameFacingLeft = "OV9281";
    public static final String kCameraNameFacingRight = "OV9281_2nd";
    // Cam mounting pose
    public static final Transform3d kRobotToCamFacingLeft = new Transform3d(
        new Translation3d(Units.inchesToMeters(4.5), Units.inchesToMeters(8.5), Units.inchesToMeters(11.75)), // 9 holes between gusset to mount Z
        new Rotation3d(0, 0, Math.toRadians(-5))
    );
    public static final Transform3d kRobotToCamFacingRight = new Transform3d(
        new Translation3d(Units.inchesToMeters(5.25), Units.inchesToMeters(10.5), Units.inchesToMeters(11.75)),
        new Rotation3d(0, 0, Math.toRadians(-40))
    );
    
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout;
    static {
        var originalLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        List<AprilTag> tags = new ArrayList<>(originalLayout.getTags());
        tags.removeIf(tag -> tag.ID <= 5 || (tag.ID >= 12 && tag.ID <= 16));
        kTagLayout = new AprilTagFieldLayout(tags, originalLayout.getFieldLength(), originalLayout.getFieldWidth());
        for (var tag : kTagLayout.getTags()) {
            System.err.println("tag id "+tag.ID);
        }
    }
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final double kSingletagBaseTrustTrlStdDevs = 1;
    public static final double kSingletagBaseTrustRotStdDevs = 15;
    public static final double kMultitagBaseTrustTrlStdDevs = 0.5;
    public static final double kMultitagBaseTrustRotStdDevs = 2;
    /** Lower values reduce trust when estimate is far from visible tags */
    public static final double kDistanceTrustScale = 5;
    /** Lower values reduce trust as robot rotates faster */
    public static final double kRotSpeedTrustScale = 0.5;
}
