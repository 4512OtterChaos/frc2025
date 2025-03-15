package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;

public class FieldUtil {
    public static final Distance kFieldWidth = Meters.of(VisionConstants.kTagLayout.getFieldWidth());
    public static final Distance kFieldLength = Meters.of(VisionConstants.kTagLayout.getFieldLength());

    public static final Translation2d kReefTrl = new Translation2d(
        Units.inchesToMeters(176.746),
        kFieldWidth.div(2).in(Meters)
    );

    public static final Distance kReefWidth = Centimeters.of(166);
    public static final Distance kReefPoleDist = Centimeters.of(33);
    

    //########## REEF ALIGNMENT

    // Stop adjusting alignment when this close:
    public static final double kAlignAdjustDrivePosTol = Inches.of(7).in(Meters);
    public static final double kAlignAdjustTurnPosTol = Degrees.of(10).in(Radians);

    public enum ReefPosition {
        CENTER,
        LEFT,
        RIGHT
    }

    private static final Translation2d kCoralScoreLeftPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(DriveConstants.kRobotLength.div(2)).in(Meters)),
        kReefTrl.getMeasureY().plus(kReefPoleDist.div(2)).in(Meters));

    
    private static final Translation2d kCoralScoreRightPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(DriveConstants.kRobotLength.div(2)).in(Meters)),
        kReefTrl.getMeasureY().minus(kReefPoleDist.div(2)).in(Meters));

    public static final List<Pose2d> kReefLeftCoralPoses = new ArrayList<Pose2d>() {{
        add(new Pose2d(kCoralScoreLeftPoseTemplate, Rotation2d.fromDegrees(0)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};

    public static final List<Pose2d> kReefRightCoralPoses = new ArrayList<Pose2d>() {{
        add(new Pose2d(kCoralScoreRightPoseTemplate, Rotation2d.fromDegrees(0)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};
    
    public static final List<Pose2d> kReefCenterPoses = new ArrayList<Pose2d>() {{
        add(new Pose2d(kCoralScoreLeftPoseTemplate, Rotation2d.fromDegrees(0)));
        add(new Pose2d(kCoralScoreRightPoseTemplate, Rotation2d.fromDegrees(0)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));

        add(new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
        add(new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};

    //########## CORAL STATION

    public static final Pose2d kRightCoralStation =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d kLeftCoralStation =
        new Pose2d(
            kRightCoralStation.getX(),
            kFieldWidth.in(Meters) - kRightCoralStation.getY(),
            Rotation2d.fromRadians(-kRightCoralStation.getRotation().getRadians()));
    public static final List<Pose2d> kCoralStationPoses = List.of(kRightCoralStation, kLeftCoralStation);

    public static Pose2d nearestCoralStation(Pose2d robotPose) {
        return robotPose.nearest(kCoralStationPoses);
    }
}
