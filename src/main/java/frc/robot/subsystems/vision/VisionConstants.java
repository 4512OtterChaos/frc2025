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
    public static final String kCameraName = "OV9281";
    // Cam mounting pose
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(4.5), Units.inchesToMeters(10), Units.inchesToMeters(11.75)), // 9 holes between gusset to mount
        new Rotation3d(0, 0, Math.toRadians(-23))
    );
    
    // The layout of the AprilTags on the field
    private static final AprilTagFieldLayout kOrigionalTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private static List<AprilTag> tags = new ArrayList<AprilTag>() {{
        addAll(kOrigionalTagLayout.getTags());
        for (AprilTag tag : kOrigionalTagLayout.getTags()){
            if (tag.ID <= 5 || (tag.ID >= 12 && tag.ID <= 16)) {
                remove(tag);
            }
        }
    }};
    public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(tags, kOrigionalTagLayout.getFieldLength(), kOrigionalTagLayout.getFieldWidth());
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final double kLowTrustTrlStdDevs = 2;
    public static final double kLowTrustRotStdDevs = Double.MAX_VALUE;
    public static final double kHighTrustTrlStdDevs = 0.5;
    public static final double kHighTrustRotStdDevs = Double.MAX_VALUE;

    public static final double kConstrainedHeadingTrust = 100.0;
}
