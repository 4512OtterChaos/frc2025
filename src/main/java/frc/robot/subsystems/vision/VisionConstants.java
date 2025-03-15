package frc.robot.subsystems.vision;

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
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final double kLowTrustTrlStdDevs = 4;
    public static final double kLowTrustRotStdDevs = Double.MAX_VALUE;
    public static final double kHighTrustTrlStdDevs = 0.5;
    public static final double kHighTrustRotStdDevs = Double.MAX_VALUE;

    public static final double kConstrainedHeadingTrust = 100.0;
}
