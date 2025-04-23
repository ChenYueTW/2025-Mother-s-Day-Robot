package frc.robot.lib.helpers;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class RebuildPhotonHelper {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Transform3d cameraPose;
    private final PhotonPoseEstimator estimator;
    private final EstimateConsumer estimateConsumer;
    private Matrix<N3, N1> curStdDevs;
    private Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0, 0, 0); // TODO
    private Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0, 0, 0); // TODO

    public RebuildPhotonHelper(
        String camName,
        Translation3d cameraTranslation, Rotation3d cameraRotation,
        EstimateConsumer estimateConsumer
    ) {
        this.camera = new PhotonCamera(camName);
        this.cameraPose = new Transform3d(cameraTranslation, cameraRotation);
        this.estimator = new PhotonPoseEstimator(this.layout, PoseStrategy.CLOSEST_TO_LAST_POSE, this.cameraPose);
        this.estimateConsumer = estimateConsumer;
    }

    public void periodic() {
        for (var change : camera.getAllUnreadResults()) {
            var pose = this.estimator.update(change);
            this.updateEstimationStdDevs(pose, change.getTargets());

            pose.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs();
                    this.estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var target : targets) {
            var tagPose = this.estimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
                estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            curStdDevs = kSingleTagStdDevs;
            return;
        }
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
