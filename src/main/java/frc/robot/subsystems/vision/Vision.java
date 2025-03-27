package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.util.TunableNumber;

public class Vision {
    private final PhotonCamera cameraLeft;
    private final PhotonCamera cameraRight;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;

    private final StructArrayPublisher<Pose3d> leftVisibleTagsPub = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Left Cam/Visible Tag Poses", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> leftEstimatePosePub = NetworkTableInstance.getDefault().getStructTopic("Vision/Left Cam/Estimated Pose", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rightVisibleTagsPub = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Right Cam/Visible Tag Poses", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> rightEstimatePosePub = NetworkTableInstance.getDefault().getStructTopic("Vision/Right Cam/Estimated Pose", Pose3d.struct).publish();
    
    private final TunableNumber lowTrustTrlStdDevs = new TunableNumber("Vision/lowTrustTrlStdDevs", kLowTrustTrlStdDevs);
    private final TunableNumber lowTrustRotStdDevs = new TunableNumber("Vision/lowTrustRotStdDevs", kLowTrustRotStdDevs);
    private Matrix<N3, N1> lowTrustStdDevs = VecBuilder.fill(kLowTrustTrlStdDevs, kLowTrustTrlStdDevs, kLowTrustRotStdDevs);
    private final TunableNumber highTrustTrlStdDevs = new TunableNumber("Vision/highTrustTrlStdDevs", kHighTrustTrlStdDevs);
    private final TunableNumber highTrustRotStdDevs = new TunableNumber("Vision/highTrustRotStdDevs", kHighTrustRotStdDevs);
    private Matrix<N3, N1> highTrustStdDevs = VecBuilder.fill(kHighTrustTrlStdDevs, kHighTrustTrlStdDevs, kHighTrustRotStdDevs);

    // private final TunableNumber constrainedHeadingTrust = new TunableNumber("Vision/constrainedHeadingTrust", kConstrainedHeadingTrust);

    // // Simulation
    private PhotonCameraSim cameraSimFacingLeft;
    private PhotonCameraSim cameraSimFacingRight;
    private VisionSystemSim visionSim;

    public Vision() {
        cameraLeft = new PhotonCamera(kCameraNameFacingLeft);
        cameraRight = new PhotonCamera(kCameraNameFacingRight);

        photonEstimator =
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamFacingLeft);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(
                1280, 800,
                MatBuilder.fill(Nat.N3(), Nat.N3(), 912.709493756531,0.0,604.8806144623338,0.0,912.1834423662752,417.52151705840043,0.0,0.0,1.0),
                VecBuilder.fill(0.04876898702521537,-0.07988383118417577,5.014752423658081E-4,-8.445052005705895E-4,0.014724717037463421,-0.001568504911944983,0.0025829433227567843,-0.0013020935054274872)
            );
            cameraProp.setCalibError(0.45, 0.10);
            cameraProp.setFPS(40);
            cameraProp.setAvgLatencyMs(30);
            cameraProp.setLatencyStdDevMs(8);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSimFacingLeft = new PhotonCameraSim(cameraLeft, cameraProp);
            cameraSimFacingLeft.setMinTargetAreaPixels(400);
            cameraSimFacingRight = new PhotonCameraSim(cameraRight, cameraProp);
            cameraSimFacingRight.setMinTargetAreaPixels(400);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSimFacingLeft, kRobotToCamFacingLeft);
            visionSim.addCamera(cameraSimFacingRight, kRobotToCamFacingRight);

        }
    }

    public void periodic() {
        changeTunable();

        // Make tags alliance relative
        if (!DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                kTagLayout.setOrigin(
                    allianceColor == Alliance.Red
                        ? OriginPosition.kRedAllianceWallRightSide
                        : OriginPosition.kBlueAllianceWallRightSide
                );
            });
        }
    }

    public void update(SwerveDrivePoseEstimator estimator, Rotation2d fieldToRobotRot, double rotationTimestamp) {
        // Update estimator for new left-facing camera results
        var leftResults = cameraLeft.getAllUnreadResults();
        if (leftResults.size() > 1) { // ensure new results are last
            leftResults.sort((r1, r2) -> Double.compare(r1.getTimestampSeconds(), r2.getTimestampSeconds()));
        }
        for (var result : leftResults) {
            estimatePoseGivenRot(result, fieldToRobotRot, kRobotToCamFacingLeft).ifPresent(estimate -> {
                var stdDevs = getEstimationStdDevs(estimate.estimatedPose);
                estimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, stdDevs);
            });
        }

        // Update estimator for new right-facing camera results
        var rightResults = cameraRight.getAllUnreadResults();
        if (rightResults.size() > 1) { // ensure new results are last
            rightResults.sort((r1, r2) -> Double.compare(r1.getTimestampSeconds(), r2.getTimestampSeconds()));
        }
        for (var result : rightResults) {
            estimatePoseGivenRot(result, fieldToRobotRot, kRobotToCamFacingRight).ifPresent(estimate -> {
                var stdDevs = getEstimationStdDevs(estimate.estimatedPose);
                estimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, stdDevs);
            });
        }

        // Grab updated pose for logging
        var updatedRobotPose = estimator.getEstimatedPosition();
        var updatedRobotPose3d = new Pose3d(updatedRobotPose);

        // Log last left-facing camera estimate and visible tags
        if (leftResults.isEmpty()) {
            leftEstimatePosePub.set(null);
            leftVisibleTagsPub.set(null);
        }
        else {
            leftVisibleTagsPub.set(leftResults.get(leftResults.size() - 1).targets.stream()
                    .map(tag -> updatedRobotPose3d.plus(kRobotToCamFacingLeft).plus(tag.bestCameraToTarget))
                    .collect(Collectors.toList()).toArray(Pose3d[]::new));
        }

        // Log last right-facing camera estimate and visible tags
        if (rightResults.isEmpty()) {
            rightEstimatePosePub.set(null);
            rightVisibleTagsPub.set(null);
        }
        else {
            rightVisibleTagsPub.set(rightResults.get(rightResults.size() - 1).targets.stream()
                    .map(tag -> updatedRobotPose3d.plus(kRobotToCamFacingRight).plus(tag.bestCameraToTarget))
                    .collect(Collectors.toList()).toArray(Pose3d[]::new));
        }
    }

    public PhotonPipelineResult getLatestResult() {
        return cameraLeft.getLatestResult();
    }

    public Optional<EstimatedRobotPose> estimatePoseGivenRot(PhotonPipelineResult result, Rotation2d fieldToRobotRot, Transform3d robotToCam) {
        var bestTarget = result.getBestTarget();
        if (bestTarget == null) return Optional.empty();
        var camToTargetTrl = bestTarget.getBestCameraToTarget().getTranslation();

        var tagPoseMaybe = kTagLayout.getTagPose(bestTarget.fiducialId);
        if (tagPoseMaybe.isEmpty()) return Optional.empty();
        var tagPose = tagPoseMaybe.get();
        Rotation3d tagRot = tagPose.getRotation();
        var fieldToCamRot = robotToCam.getRotation().rotateBy(new Rotation3d(fieldToRobotRot));
        var targetToCamTrl = camToTargetTrl.unaryMinus().rotateBy(
                fieldToCamRot.minus(tagRot));

        var targetToCam = new Transform3d(
            tagPose,
            new Pose3d(
                tagPose.getTranslation().plus(targetToCamTrl.rotateBy(tagPose.getRotation())),
                fieldToCamRot
            )
        );

        var estimate = new EstimatedRobotPose(
            tagPose.plus(targetToCam).plus(robotToCam.inverse()),
            result.getTimestampSeconds(),
            List.of(bestTarget),
            null
        );
        return Optional.of(estimate);
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose3d estimatedPose) {
        var estStdDevs = lowTrustStdDevs;
        var result = getLatestResult();
        var targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = highTrustStdDevs;
        // Increase std devs based on (average) distance
        if ((numTags == 1 && avgDist > 4) || !MathUtil.isNear(0, estimatedPose.getZ(), 0.5))
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public void changeTunable() {
        lowTrustTrlStdDevs.poll();
        lowTrustRotStdDevs.poll();
        highTrustTrlStdDevs.poll();
        highTrustRotStdDevs.poll();
        // constrainedHeadingTrust.poll();

        int hash = hashCode();
        if (lowTrustTrlStdDevs.hasChanged(hash) || lowTrustRotStdDevs.hasChanged(hash)) {
            lowTrustStdDevs = VecBuilder.fill(lowTrustTrlStdDevs.get(), lowTrustTrlStdDevs.get(), lowTrustRotStdDevs.get());
        }
        if (highTrustTrlStdDevs.hasChanged(hash) || highTrustRotStdDevs.hasChanged(hash)) {
            highTrustStdDevs = VecBuilder.fill(highTrustTrlStdDevs.get(), highTrustTrlStdDevs.get(), highTrustRotStdDevs.get());
        }
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose.relativeTo(kTagLayout.getOrigin().toPose2d()));
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
