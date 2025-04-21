package frc.robot.lib.helpers;

import java.util.function.Supplier;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.math.AprilTagPoseEstimator;

public class PhotonHelper {
    private final PhotonCamera photonCamera;
    private final AprilTagPoseEstimator aprilTagPoseEstimator;

    public PhotonHelper(String cameraName,
        Vector3D cameraPose, Vector3D centralSight, Vector3D xAxis, Vector3D yAxis,
        double torlerance, Supplier<Double> gyroAngle) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.aprilTagPoseEstimator = new AprilTagPoseEstimator(
            cameraPose, centralSight, xAxis, yAxis, torlerance, gyroAngle);
    }

    public PhotonTrackedTarget getTargetFromId(int id) {
        var result = this.photonCamera.getLatestResult();
        if (!result.hasTargets()) return null;
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == id) return target;
        }
        return null;
    }

    public boolean hasTargetId(int id) {
        var result = this.photonCamera.getLatestResult();
        if (!result.hasTargets()) return false;
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == id) return true;
        }
        return false;
    }

    public double getTx() {
        var result = this.photonCamera.getLatestResult();
        if (!result.hasTargets()) return -1;
        return result.getBestTarget().getYaw();
    }

    public double getTy() {
        var result = this.photonCamera.getLatestResult();
        if (!result.hasTargets()) return -1;
        return result.getBestTarget().getPitch();
    }

    public int getAprilTagId() {
        var result = this.photonCamera.getLatestResult();
        if (!result.hasTargets()) return -1;
        return result.getBestTarget().getFiducialId();
    }

    public Translation3d getAprilTagPose() {
        int id = this.getAprilTagId();
        if (id == -1 || id == -2) return null;
        return this.aprilTagPoseEstimator.getAprilTagPose(this.getTx(), this.getTy(), id);
    }

    public Translation3d getAprilTagPose(double tx, double ty, int id) {
        return this.aprilTagPoseEstimator.getAprilTagPose(tx, ty, id);
    }

    public Rotation2d getAprilTagRotation() {
        int id = this.getAprilTagId();
        if (id == -1 || id == -2) return null;
        return this.aprilTagPoseEstimator.getAprilTagRotation(id);
    }

    public Rotation2d getAprilTagRotationFromId(int id) {
        return this.aprilTagPoseEstimator.getAprilTagRotation(id);
    }

    public String getTargetString() {
        return "Tx: " + this.getTx() + " Ty: " + this.getTy() + " Id: " + this.getAprilTagId();
    }

    public String getApriltagPoseString() {
        if (!this.hasTarget()) return new Translation3d().toString();
        return this.aprilTagPoseEstimator.getAprilTagPose(this.getTx(), this.getTy(), this.getAprilTagId()).toString();
    }

    public boolean hasTarget() {
        return this.photonCamera.getLatestResult().hasTargets();
    }
}
