package frc.robot.subsystems.vision;

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
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final double kLowTrustTrlStdDevs = 2;
    public static final double kLowTrustRotStdDevs = Double.MAX_VALUE;
    public static final double kHighTrustTrlStdDevs = 0.5;
    public static final double kHighTrustRotStdDevs = Double.MAX_VALUE;

    // public static final double kConstrainedHeadingTrust = 100.0;
}
