package frc.robot.lib.camera;

import java.util.function.Supplier;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.lib.math.AprilTagPoseEstimator;

public class LimelightHelper {
    private final DoubleSubscriber aprilTagId;
    private final DoubleSubscriber ty;
    private final DoubleSubscriber tx;
    private final AprilTagPoseEstimator aprilTagPoseEstimator;

    public LimelightHelper(
        String tableName,
        Vector3D cameraPose, Vector3D centralSight, Vector3D xAxis, Vector3D yAxis,
        double tolerance, Supplier<Double> gyroAngle) {
        this.tx = NetworkTableInstance.getDefault().getTable(tableName).getDoubleTopic("tx").subscribe(-1);
        this.ty = NetworkTableInstance.getDefault().getTable(tableName).getDoubleTopic("ty").subscribe(-1);
        this.aprilTagId = NetworkTableInstance.getDefault().getTable(tableName).getDoubleTopic("tid").subscribe(-1);

        this.aprilTagPoseEstimator = new AprilTagPoseEstimator(
            cameraPose, centralSight, xAxis, yAxis, tolerance, gyroAngle);
    }

    public int getAprilTagId() {
        return (int) this.aprilTagId.get();
    }

    public double getTx() {
        return this.tx.get();
    }

    public double getTy() {
        return this.ty.get();
    }

    public boolean hasTarget() {
        return this.getAprilTagId() != 0;
    }

    public Translation3d getAprilTagPose() {
        if (this.getAprilTagId() == -1) return null;
        return this.aprilTagPoseEstimator.getAprilTagPose(this.getTx(), this.getTy(), this.getAprilTagId());
    }

    public String getTargetString() {
        return String.format("Tx: %d, Ty: %d, Id: %i", this.getTx(), this.getTy(), this.getAprilTagId());
    }

    public String getApriltagPoseString() {
        if (this.getTx() == -1 || this.getTy() == -1 || this.getAprilTagId() == -1) return "NULL";
        return this.aprilTagPoseEstimator.getAprilTagPose(this.getTx(), this.getTy(), this.getAprilTagId()).toString();
    }
}
