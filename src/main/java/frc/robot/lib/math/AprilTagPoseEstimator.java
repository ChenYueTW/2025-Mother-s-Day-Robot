package frc.robot.lib.math;

import java.util.function.Supplier;

import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class AprilTagPoseEstimator {
    private final double TOLERANCE;
    private final Vector3D CAMERA_POSE;
    private final Vector3D CENTRAL_SIGHT;
    private final Vector3D CAM_X_AXIS;
    private final Vector3D CAM_Y_AXIS;
    @SuppressWarnings("unused")
    private final Supplier<Double> gyroAngle;

    public AprilTagPoseEstimator(
        Vector3D cameraPose, Vector3D centralSight, Vector3D xAxis, Vector3D yAxis,
        double tolerance, Supplier<Double> gyroAngle) {
        this.CAMERA_POSE = cameraPose;
        this.CENTRAL_SIGHT = centralSight;
        this.CAM_X_AXIS = xAxis;
        this.CAM_Y_AXIS = yAxis;
        this.TOLERANCE = tolerance;
        this.gyroAngle = gyroAngle;
    }

    public Translation3d getAprilTagPose(double tx, double ty, int id) {
        if (id == -1) return new Translation3d(0.0, 0.0, 0.0);
        double apriltagHeight = AprilTagPose.getTranslation3d(id).getZ();
        Plane aprilTagPlane = new Plane(new Vector3D(0.0, 0.0, apriltagHeight), new Vector3D(0.0, 0.0, 1.0), this.TOLERANCE);

        Rotation xRot = new Rotation(this.CAM_Y_AXIS, -Units.degreesToRadians(tx), RotationConvention.VECTOR_OPERATOR);
        Rotation yRot = new Rotation(this.CAM_X_AXIS, Units.degreesToRadians(ty), RotationConvention.VECTOR_OPERATOR);
        Vector3D xVector = xRot.applyTo(this.CENTRAL_SIGHT);
        Vector3D yVector = yRot.applyTo(this.CENTRAL_SIGHT);

        Plane xPlane = new Plane(this.CAMERA_POSE, this.CAMERA_POSE.add(xVector), this.CAMERA_POSE.add(this.CAM_Y_AXIS), this.TOLERANCE);
        Plane yPlane = new Plane(this.CAMERA_POSE, this.CAMERA_POSE.add(yVector), this.CAMERA_POSE.add(this.CAM_X_AXIS), this.TOLERANCE);
        Vector3D intersect = Plane.intersection(xPlane, yPlane, aprilTagPlane);

        // Translation2d rotatedVec = MathHelper.rotation2dMatrix(new Translation2d(intersect.getX(), intersect.getY()), this.gyroAngle.get());
        
        return new Translation3d(intersect.getX(), intersect.getY(), intersect.getZ());
    }

    public Rotation2d getAprilTagRotation(int id) {
        return AprilTagPose.getRotation2d(id);
    }
}
