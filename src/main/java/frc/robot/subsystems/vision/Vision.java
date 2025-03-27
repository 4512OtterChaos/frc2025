package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
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
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.util.TunableNumber;

public class Vision {
    private final PhotonCamera cameraLeft = new PhotonCamera(kCameraNameFacingLeft);
    private final PhotonCamera cameraRight = new PhotonCamera(kCameraNameFacingRight);
    private final TimeInterpolatableBuffer<Rotation2d> headingBuffer =
            TimeInterpolatableBuffer.createBuffer(1.0);

    private final StructPublisher<Pose3d> leftCamPose = NetworkTableInstance.getDefault().getStructTopic("Vision/Left Cam/RobotToCam Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> rightCamPose = NetworkTableInstance.getDefault().getStructTopic("Vision/Right Cam/RobotToCam Pose", Pose3d.struct).publish();

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

    //----- Simulation
    private PhotonCameraSim cameraSimFacingLeft;
    private PhotonCameraSim cameraSimFacingRight;
    private VisionSystemSim visionSim;
    {
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var leftCamProp = new SimCameraProperties();
            leftCamProp.setCalibration(
                1280, 800,
                MatBuilder.fill(Nat.N3(), Nat.N3(), 912.709493756531,0.0,604.8806144623338,0.0,912.1834423662752,417.52151705840043,0.0,0.0,1.0),
                VecBuilder.fill(0.04876898702521537,-0.07988383118417577,5.014752423658081E-4,-8.445052005705895E-4,0.014724717037463421,-0.001568504911944983,0.0025829433227567843,-0.0013020935054274872)
            );
            leftCamProp.setCalibError(0.45, 0.10);
            leftCamProp.setFPS(40);
            leftCamProp.setAvgLatencyMs(30);
            leftCamProp.setLatencyStdDevMs(8);

            var rightCamProp = leftCamProp.copy();
            rightCamProp.setCalibration(
                1280, 800,
                MatBuilder.fill(Nat.N3(), Nat.N3(), 901.0021345114463,0.0,664.4681708224546,0.0,900.9492509895897,426.3018086357822,0.0,0.0,1.0),
                VecBuilder.fill(0.045596409600450645,-0.055142592429464926,-8.63711484780898E-5,-7.915368273629485E-4,-0.007584370524445873,-0.0012547761325422994,0.005980279080816732,0.00159536717183096)
            );

            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSimFacingLeft = new PhotonCameraSim(cameraLeft, leftCamProp);
            cameraSimFacingLeft.setMinTargetAreaPixels(400);
            cameraSimFacingRight = new PhotonCameraSim(cameraRight, leftCamProp);
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

    /**
     * Updates the given pose estimator with all unread vision measurements from the cameras.
     * @param estimator
     * @param fieldToRobotRot Heading of the robot on the field
     * @param rotationTimestamp Timestamp the heading of the robot was captured at
     */
    public void update(SwerveDrivePoseEstimator estimator, Rotation2d fieldToRobotRot, double rotationTimestamp) {
        headingBuffer.addSample(rotationTimestamp, fieldToRobotRot);

        // Update estimator for new left-facing camera results
        var leftResults = cameraLeft.getAllUnreadResults();
        processResults(leftResults, estimator, kRobotToCamFacingLeft).ifPresent(latestEstimate -> {
            leftEstimatePosePub.set(latestEstimate);
        });

        // Update estimator for new right-facing camera results
        var rightResults = cameraRight.getAllUnreadResults();
        processResults(rightResults, estimator, kRobotToCamFacingRight).ifPresent(latestEstimate -> {
            rightEstimatePosePub.set(latestEstimate);
        });

        // Grab updated pose for logging
        var updatedRobotPose = estimator.getEstimatedPosition();
        var updatedRobotPose3d = new Pose3d(updatedRobotPose);
        leftCamPose.set(updatedRobotPose3d.plus(kRobotToCamFacingLeft));
        rightCamPose.set(updatedRobotPose3d.plus(kRobotToCamFacingRight));

        // Log last left-facing camera estimate and visible tags
        if (leftResults.isEmpty()) {
            leftVisibleTagsPub.set(null);
        }
        else {
            leftVisibleTagsPub.set(leftResults.get(leftResults.size() - 1).targets.stream()
                    .map(tag -> updatedRobotPose3d.plus(kRobotToCamFacingLeft).plus(tag.bestCameraToTarget))
                    .collect(Collectors.toList()).toArray(Pose3d[]::new));
        }

        // Log last right-facing camera estimate and visible tags
        if (rightResults.isEmpty()) {
            rightVisibleTagsPub.set(null);
        }
        else {
            rightVisibleTagsPub.set(rightResults.get(rightResults.size() - 1).targets.stream()
                    .map(tag -> updatedRobotPose3d.plus(kRobotToCamFacingRight).plus(tag.bestCameraToTarget))
                    .collect(Collectors.toList()).toArray(Pose3d[]::new));
        }
    }

    /**
     * For all given camera results, add pose estimates to the given estimator.
     * @param results
     * @param estimator
     * @param robotToCam
     * @return The estimated robot pose for the last given result, or empty if none.
     */
    private Optional<Pose3d> processResults(List<PhotonPipelineResult> results, SwerveDrivePoseEstimator estimator, Transform3d robotToCam) {
        Optional<Pose3d> latestCamEstimate = Optional.empty();

        if (results.size() > 1) { // ensure new results are last
            results.sort((r1, r2) -> Double.compare(r1.getTimestampSeconds(), r2.getTimestampSeconds()));
        }
        for (var result : results) {
            Rotation2d rotAtResult = headingBuffer.getSample(result.getTimestampSeconds()).get();
            var opt = estimatePoseGivenRot(result, rotAtResult, robotToCam);
            if (opt.isEmpty()) continue;
            EstimatedRobotPose estimate = opt.get();
            latestCamEstimate = Optional.of(estimate.estimatedPose);
            var stdDevs = getEstimationStdDevs(result, estimate.estimatedPose);
            estimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, stdDevs);
        }

        return latestCamEstimate;
    }

    /**
     * Find the estimated robot pose for the given camera result.
     * Ignores estimated rotation in favor of the given robot rotation, using only estimated translation and tag layout.
     * @param result
     * @param fieldToRobotRot
     * @param robotToCam
     * @return
     */
    public Optional<EstimatedRobotPose> estimatePoseGivenRot(PhotonPipelineResult result, Rotation2d fieldToRobotRot, Transform3d robotToCam) {
        /*
         * We will always use a single tag for estimating the robot's pose. In the case that multiple tags
         * are visible, the multitag calculation on-coprocessor should update all individual tag transforms,
         * meaning any tag we choose should produce the same result.
         */
        // First, discard any visible tags we aren't tracking in our tag layout
        var visibleTags = result.getTargets();
        visibleTags.removeIf(target -> kTagLayout.getTagPose(target.fiducialId).isEmpty());
        if (visibleTags.isEmpty()) return Optional.empty();

        var target = visibleTags.get(0);
        var camToTargetTrl = target.getBestCameraToTarget().getTranslation();

        /*
         * Traditionally, we want to use the camera-to-tag transform off of the tag layout pose to find the camera's
         * pose in the field. In this case, we will replace the transform's rotation with our given robot rotation
         * to reduce noise in the estimate translation caused by high noise in the estimated rotation.
         */
        var tagPose = kTagLayout.getTagPose(target.fiducialId).get();
        Rotation3d tagRot = tagPose.getRotation();
        var fieldToCamRot = robotToCam.getRotation().rotateBy(new Rotation3d(fieldToRobotRot));
        var targetToCamTrl = camToTargetTrl.unaryMinus().rotateBy(
                fieldToCamRot.minus(tagRot));

        Pose3d fieldToCam = new Pose3d(
            tagPose.getTranslation().plus(targetToCamTrl.rotateBy(tagPose.getRotation())),
            fieldToCamRot
        );


        var estimate = new EstimatedRobotPose(
            fieldToCam.plus(robotToCam.inverse()),
            result.getTimestampSeconds(),
            result.getTargets(),
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
    public Matrix<N3, N1> getEstimationStdDevs(PhotonPipelineResult result, Pose3d estimatedPose) {
        var estStdDevs = lowTrustStdDevs;
        var targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
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
