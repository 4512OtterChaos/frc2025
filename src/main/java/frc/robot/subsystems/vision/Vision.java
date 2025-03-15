package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Optional;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.util.TunableNumber;

public class Vision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;

    private final StructPublisher<Pose3d> pipelinePosePub = NetworkTableInstance.getDefault().getStructTopic("Vision/Pipeline Pose", Pose3d.struct).publish();

    private final TunableNumber lowTrustTrlStdDevs = new TunableNumber("Vision/lowTrustTrlStdDevs", kLowTrustTrlStdDevs);
    private final TunableNumber lowTrustRotStdDevs = new TunableNumber("Vision/lowTrustRotStdDevs", kLowTrustRotStdDevs);
    private Matrix<N3, N1> lowTrustStdDevs = VecBuilder.fill(kLowTrustTrlStdDevs, kLowTrustTrlStdDevs, kLowTrustRotStdDevs);
    private final TunableNumber highTrustTrlStdDevs = new TunableNumber("Vision/highTrustTrlStdDevs", kHighTrustTrlStdDevs);
    private final TunableNumber highTrustRotStdDevs = new TunableNumber("Vision/highTrustRotStdDevs", kHighTrustRotStdDevs);
    private Matrix<N3, N1> highTrustStdDevs = VecBuilder.fill(kHighTrustTrlStdDevs, kHighTrustTrlStdDevs, kHighTrustRotStdDevs);

    private final TunableNumber constrainedHeadingTrust = new TunableNumber("Vision/constrainedHeadingTrust", kConstrainedHeadingTrust);

    // // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public Vision() {
        camera = new PhotonCamera(kCameraName);

        photonEstimator =
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
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
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(45);
            cameraProp.setAvgLatencyMs(25);
            cameraProp.setLatencyStdDevMs(8);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, kRobotToCam);

            cameraSim.enableDrawWireframe(false);
        }
    }

    public void periodic() {
        changeTunable();
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Rotation2d fieldRotation, double timestampRotation) {
        if (!DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                kTagLayout.setOrigin(
                    allianceColor == Alliance.Red
                        ? OriginPosition.kRedAllianceWallRightSide
                        : OriginPosition.kBlueAllianceWallRightSide
                );
            });
        }

        var latest = getLatestResult();
        photonEstimator.addHeadingData(timestampRotation, fieldRotation);
        var visionEst = photonEstimator.update(
            latest,
            camera.getCameraMatrix(),
            camera.getDistCoeffs(),
            Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, constrainedHeadingTrust.get()))
        );
        double latestTimestamp = latest.getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        visionEst.ifPresentOrElse(
            (est)->pipelinePosePub.set(est.estimatedPose),
            ()->pipelinePosePub.set(null)
        );
        if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est -> {
                        getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d());
                    },
                    () -> {
                        if (newResult) {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        }
                    });
        }
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
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
        constrainedHeadingTrust.poll();

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
