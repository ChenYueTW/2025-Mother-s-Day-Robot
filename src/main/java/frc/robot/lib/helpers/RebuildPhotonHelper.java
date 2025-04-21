package frc.robot.lib.helpers;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class RebuildPhotonHelper {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Transform3d cameraPose;
    private final PhotonPoseEstimator estimator;

    public RebuildPhotonHelper(
        String camName,
        Translation3d cameraTranslation, Rotation3d cameraRotation
    ) {
        this.camera = new PhotonCamera(camName);
        this.cameraPose = new Transform3d(cameraTranslation, cameraRotation);
        this.estimator = new PhotonPoseEstimator(this.layout, PoseStrategy.CLOSEST_TO_LAST_POSE, this.cameraPose);
    }
}
