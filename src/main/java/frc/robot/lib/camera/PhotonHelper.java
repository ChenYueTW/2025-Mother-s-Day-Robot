package frc.robot.lib.camera;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.math.PoseTransform;

public class PhotonHelper {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final double PITCH;
    private final double YAW;
    private final PoseTransform poseTransform;
    private int tagId = -1;

    public PhotonHelper(
        String camName,
        Translation3d cameraTranslation,
        double pitch, double yaw
    ) {
        this.camera = new PhotonCamera(camName);
        this.poseTransform = new PoseTransform(new Vector3D(cameraTranslation.getX(), cameraTranslation.getY(), cameraTranslation.getZ()));
        this.PITCH = pitch;
        this.YAW = yaw;
    }

    public Translation3d getRobotToTag() {
        var result = this.camera.getLatestResult();
        if (result == null || !result.hasTargets()) return null;
        PhotonTrackedTarget target = result.getBestTarget();
        if (target == null) return null;

        this.tagId = target.fiducialId;
        Transform3d cameraToTag = target.getBestCameraToTarget();
        return this.poseTransform.getTransform(cameraToTag, this.PITCH, this.YAW);
    }

    public Pose2d getRobotToTagPose() {
        Translation3d translation3d = this.getRobotToTag();
        if (translation3d == null) return new Pose2d();
        return new Pose2d(translation3d.toTranslation2d(), new Rotation2d());
    }

    public Pose2d getRobotField() {
        Translation3d translation3d = this.getRobotToTag();
        if (translation3d == null) return new Pose2d(0.0, 0.0, new Rotation2d());
        Transform3d tagToRobot = new Transform3d(translation3d, new Rotation3d(0.0, 0.0, Math.PI));
        
        Pose3d fieldToTag = this.layout.getTagPose(this.tagId).get();
        Pose3d fieldToRobot = fieldToTag.transformBy(tagToRobot);

        return fieldToRobot.toPose2d();
    }

    public double getTimeStamp() {
        var result = this.camera.getLatestResult();
        if (result == null) return -1;
        return result.getTimestampSeconds();
    }

    public boolean hasTarget() {
        var result = this.camera.getLatestResult();
        if (result == null) return false;
        return result.hasTargets();
    }

    public Pose2d getFieldToTagPose() {
        var result = this.camera.getLatestResult();
        if (result == null) return null;
        PhotonTrackedTarget target = result.getBestTarget();
        if (target == null) return null;
        return this.layout.getTagPose(target.getFiducialId()).get().toPose2d();
    }

    public interface MeasurementProvider {
        void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds);   
    }
}