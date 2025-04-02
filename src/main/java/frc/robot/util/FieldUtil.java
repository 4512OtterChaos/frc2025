package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DriveConstants.kRobotLength;

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
    
    public static Pose2d mirrorY(Pose2d pose) {
        return new Pose2d(
            pose.getX(),
            kFieldWidth.minus(pose.getMeasureY()).in(Meters),
            pose.getRotation().unaryMinus()
        );
    }

    //########## REEF ALIGNMENT

    // Stop adjusting alignment when this close:
    public static final double kAlignAdjustDrivePosTol = Inches.of(7).in(Meters);
    public static final double kAlignAdjustTurnPosTol = Degrees.of(10).in(Radians);

    public enum Alignment {
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

    private static final Translation2d kReefCenterPoseTemplate = new Translation2d(
        kReefTrl.getX() - (kReefWidth.div(2).plus(DriveConstants.kRobotLength.div(2)).in(Meters)),
        kReefTrl.getMeasureY().in(Meters));

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
        add(new Pose2d(kReefCenterPoseTemplate, Rotation2d.fromDegrees(0)));
        add(new Pose2d(kReefCenterPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(60)), Rotation2d.fromDegrees(60)));
        add(new Pose2d(kReefCenterPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(120)));
        add(new Pose2d(kReefCenterPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));
        add(new Pose2d(kReefCenterPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(240)), Rotation2d.fromDegrees(240)));
        add(new Pose2d(kReefCenterPoseTemplate.rotateAround(kReefTrl, Rotation2d.fromDegrees(300)), Rotation2d.fromDegrees(300)));
    }};

    public static Pose2d nearestReefPose(Pose2d robotPose, Alignment reef) {
        List<Pose2d> poses;
        switch (reef) {
            case LEFT -> poses = kReefLeftCoralPoses;
            case RIGHT -> poses = kReefRightCoralPoses;
            default -> poses = kReefCenterPoses;
        }
        return robotPose.nearest(poses);
    } 

    //########## CORAL STATION

    public enum CoralStation {
        RIGHT(
            new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90))
        ),
        LEFT(
            new Pose2d(
                RIGHT.getPose().getX(),
                kFieldWidth.in(Meters) - RIGHT.getPose().getY(),
                Rotation2d.fromRadians(-RIGHT.getPose().getRotation().getRadians()))
        );

        Pose2d pose;

        private CoralStation(Pose2d pose){
            this.pose = pose;
        }

        public Pose2d getPose(){
            return this.pose;
        }
    }
    public static final List<Pose2d> kCoralStationPoses = List.of(CoralStation.RIGHT.getPose(), CoralStation.LEFT.getPose());

    private static final Transform2d kCoralStationOffsetX = new Transform2d(
        kRobotLength.div(2).in(Meters),
        0,
        Rotation2d.kZero
    );
    private static final Transform2d kCoralStationOffsetY = new Transform2d(
        0,
        Units.feetToMeters(2),
        Rotation2d.kZero
    );

    public static Pose2d nearestCoralStation(Pose2d robotPose) {
        return robotPose.nearest(kCoralStationPoses);
    }

    public static Pose2d offsetCoralStation(Pose2d station, Alignment alignment) {
        station = station.plus(kCoralStationOffsetX);
        switch (alignment) {
            case LEFT -> station = station.plus(kCoralStationOffsetY);
            case RIGHT -> station = station.plus(kCoralStationOffsetY.inverse());
            default -> {}
        }
        return station;
    }

    public static Pose2d nearestOffsetCoralStation(Pose2d robotPose, Alignment alignment) {
        return offsetCoralStation(nearestCoralStation(robotPose), alignment);
    }

    public enum AlgaeHeight {
        L2,
        L3
    }

    public enum ReefFace {
        FRONT(AlgaeHeight.L3, Rotation2d.fromDegrees(0)),
        NEARLEFT(AlgaeHeight.L2, Rotation2d.fromDegrees(-60)),
        FARLEFT(AlgaeHeight.L3, Rotation2d.fromDegrees(-120)),
        BACK(AlgaeHeight.L2, Rotation2d.fromDegrees(-180)),
        FARRIGHT(AlgaeHeight.L3, Rotation2d.fromDegrees(-240)),
        NEARRIGHT(AlgaeHeight.L2, Rotation2d.fromDegrees(-300));

        public final AlgaeHeight algaeHeight; 
        public final Pose2d centerPose;
        public final Pose2d leftPose;
        public final Pose2d rightPose;

        private ReefFace(AlgaeHeight algaeHeight, Rotation2d reefAngle){
            this.algaeHeight = algaeHeight;
            this.centerPose = new Pose2d(kReefCenterPoseTemplate.rotateAround(kReefTrl, reefAngle), reefAngle);
            this.leftPose = new Pose2d(kCoralScoreLeftPoseTemplate.rotateAround(kReefTrl, reefAngle), reefAngle);
            this.rightPose = new Pose2d(kCoralScoreRightPoseTemplate.rotateAround(kReefTrl, reefAngle), reefAngle);
        }

        public Pose2d getAlignmentPose(Alignment alignment) {
            Pose2d pose;
            switch (alignment) {
                case LEFT -> pose = leftPose;
                case RIGHT -> pose = rightPose;
                default -> pose = centerPose;
            }
            return pose;
        }

        public static ReefFace getClosest(Pose2d currentPose){
            ReefFace closest = ReefFace.FRONT;
            double closestDist = Double.MAX_VALUE;
            for (ReefFace reefFace : values()) {
                double dist = reefFace.centerPose.getTranslation().getDistance(currentPose.getTranslation());
                if (dist < closestDist) {
                    closestDist = dist;
                    closest = reefFace;
                }
            }
            return closest;
        }
    }
}
