package frc.robot.subsystems;

import java.util.function.Supplier;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.helpers.LimelightHelper;
import frc.robot.lib.helpers.PhotonHelper;
import frc.robot.lib.subsystems.SubsystemBase;
import frc.robot.lib.math.AprilTagPose;

public class VisionSubsystem extends SubsystemBase {
    public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private final PhotonHelper leftCamera;
    private final PhotonHelper rightCamera;
    private final LimelightHelper limelight;
    public Pose2d lastEstPose = new Pose2d();
    public Pose2d lastPose = null;
    public int lastId = -1;
    public int lastTagId = -1;
    public static int trackId = 0;

    public VisionSubsystem(Supplier<Double> gyroAngle) {
        super("Vision");
        this.leftCamera = new PhotonHelper(
            "ReefLeft",
            new Vector3D(-0.184286, -0.274155, 0.40209),
            new Vector3D(-1.417174903235011, 0.469981080528552, -0.231943231157464),
            new Vector3D(0.469981080528552, 1.417174903235011, 0.0),
            new Vector3D(-0.328704126171595, 0.109008930400669, 2.229266922413948),
            0.0,
            gyroAngle);
        this.rightCamera = new PhotonHelper(
            "ReefRight",
            new Vector3D(-0.184286, 0.274155, 0.40209),
            new Vector3D(-1.375418205431189, -0.444548193124036, -0.175920717441593),
            new Vector3D(-0.444548193124036, 1.375418205431189, 0.0),
            new Vector3D(-0.241964557481683, -0.078205237071744, 2.089398335841397),
            0.0,
            gyroAngle);
        this.limelight = new LimelightHelper(
            "BargeLimelight",
            new Vector3D(0.083819, -0.279558, 0.932883),
            new Vector3D(0.097288, 0.0,0.081634),
            new Vector3D(0.0, -0.097288, 0.0),
            new Vector3D(-0.07942008592, 0, 0.00946495444),
            0.0,
            gyroAngle);
    }

    public Translation3d getLeftApriltagPose() {
        return this.leftCamera.getAprilTagPose();
    }

    public Translation3d getRightApriltagPose() {
        return this.rightCamera.getAprilTagPose();
    }

    public Translation3d getBargeLimelightApriltagPose() {
        return this.limelight.getAprilTagPose();
    }

    public Pose2d getAprilTagPose2d() {
        Pose2d est = this.getAprilTagPose();
        if (est != null) {
            lastPose = est;
            return est;
        }

        if (lastPose != null) return lastPose;
        return null;
    }

    public Pose2d getAprilTagFieldPose() {
        int leftCameraId = this.leftCamera.getAprilTagId();
        int rightCameraId = this.rightCamera.getAprilTagId();
        int id;

        if (leftCameraId < 0 && rightCameraId < 0) return null;
        if (leftCameraId != rightCameraId && leftCameraId >= 0 && rightCameraId >= 0) return null;

        id = (rightCameraId < 0) ? leftCameraId : rightCameraId;

        lastId = id;
        return new Pose2d(AprilTagPose.getTranslation3d(id).toTranslation2d(), AprilTagPose.getRotation2d(id));
    }

    public Pose2d getAprilTagFieldPoseFromId(int id) {
        PhotonTrackedTarget leftTarget = this.leftCamera.getTargetFromId(id);
        PhotonTrackedTarget rightTarget = this.rightCamera.getTargetFromId(id);
        int tagId;

        if (leftTarget == null && rightTarget == null) return null;

        tagId = leftTarget != null ? leftTarget.getFiducialId() : rightTarget.getFiducialId();

        lastTagId = tagId;
        return new Pose2d(AprilTagPose.getTranslation3d(tagId).toTranslation2d(), AprilTagPose.getRotation2d(tagId));
    }

    public Pose2d getAprilTagFieldPoseFromLastUpdate() {
        Pose2d pose = this.getAprilTagFieldPose();
        if (pose != null) return pose;
        if (lastId >= 0) return new Pose2d(AprilTagPose.getTranslation3d(lastId).toTranslation2d(), AprilTagPose.getRotation2d(lastId));
        return null;
    }

    public Pose2d getAprilTagFieldPoseFromLastUpdateWithId(int id) {
        Pose2d pose = this.getAprilTagFieldPoseFromId(id);
        if (pose != null) return pose;
        if (lastTagId >= 0) return new Pose2d(AprilTagPose.getTranslation3d(lastTagId).toTranslation2d(), AprilTagPose.getRotation2d(lastTagId));
        return null;
    }

    public Pose2d getAprilTagPose() {
        Translation3d leftCameraPose = this.leftCamera.getAprilTagPose();
        Translation3d rightCameraPose = this.rightCamera.getAprilTagPose();
        if (leftCameraPose == null && rightCameraPose == null) return null;

        Translation3d finalPose;

        if (leftCameraPose == null) {
            finalPose = rightCameraPose;
        } else {
            if (rightCameraPose == null) {
                finalPose = leftCameraPose;
            } else {
                finalPose = leftCameraPose.plus(rightCameraPose).div(2.0);
            }
        }

        Rotation2d rotation = this.getAprilTagRotation();
        if (rotation == null) return null;

        return new Pose2d(finalPose.toTranslation2d(), rotation);
    }

    public Pose2d getAprilTagPoseFromId(int id) {
        PhotonTrackedTarget leftCameraTarget = this.leftCamera.getTargetFromId(id);
        PhotonTrackedTarget rightCameraTarget = this.rightCamera.getTargetFromId(id);
        if (leftCameraTarget == null && rightCameraTarget == null) return null;

        Translation3d finalPose;

        if (leftCameraTarget == null) {
            finalPose = this.rightCamera.getAprilTagPose(rightCameraTarget.getYaw(), rightCameraTarget.getPitch(), rightCameraTarget.getFiducialId());
        } else {
            if (rightCameraTarget == null) {
                finalPose = this.leftCamera.getAprilTagPose(leftCameraTarget.getYaw(), leftCameraTarget.getPitch(), leftCameraTarget.getFiducialId());
            } else {
                finalPose = this.leftCamera.getAprilTagPose(leftCameraTarget.getYaw(), leftCameraTarget.getPitch(), leftCameraTarget.getFiducialId())
                    .plus(this.rightCamera.getAprilTagPose(rightCameraTarget.getYaw(), rightCameraTarget.getPitch(), rightCameraTarget.getFiducialId()))
                    .div(2.0);
            }
        }

        Rotation2d rotaiton = this.getAprilTagRotationFromId(id);
        if (rotaiton == null) return null;
        return new Pose2d(finalPose.toTranslation2d(), rotaiton);
    }

    public Rotation2d getAprilTagRotation() {
        return (this.leftCamera.hasTarget()) ?
            this.leftCamera.getAprilTagRotation() : this.rightCamera.getAprilTagRotation();
    }

    public Rotation2d getAprilTagRotationFromId(int id) {
        return (this.leftCamera.hasTargetId(id)) ?
            this.leftCamera.getAprilTagRotation() : this.rightCamera.getAprilTagRotation();
    }

    public boolean leftHasTarget() {
        return this.leftCamera.hasTarget();
    }

    public boolean rightHasTarget() {
        return this.rightCamera.hasTarget();
    }

    public boolean limelightHasTarget() {
        return this.limelight.hasTarget();
    }

    @Override
    public void putDashboard() {
        // SmartDashboard.putString("EstimatedGlobalPose", this.leftCamera.getEstimatedGlobalPose(lastEstPose).toString());
        SmartDashboard.putString("Camera/lastPose", this.lastPose == null ? "NULL" : this.lastPose.toString());
        SmartDashboard.putString("Camera/leftPose", this.leftCamera.getApriltagPoseString());
        SmartDashboard.putString("Camera/leftTargetPose", this.leftCamera.getTargetString());
        SmartDashboard.putBoolean("Camera/leftHasTarget", this.leftCamera.hasTarget());
        SmartDashboard.putString("Camera/rightTargetPose", this.rightCamera.getTargetString());
        SmartDashboard.putString("Camera/rightPose", this.rightCamera.getApriltagPoseString());
        // SmartDashboard.putString("Camera/TTTTTTTTTTTT", this.getAprilTagFieldPose().toString());
    }
}
