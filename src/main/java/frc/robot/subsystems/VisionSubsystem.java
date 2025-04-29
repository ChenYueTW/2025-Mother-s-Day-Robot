package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.camera.PhotonHelper;
import frc.robot.lib.camera.PhotonHelper.MeasurementProvider;
import frc.robot.lib.math.AprilTagPose;
import frc.robot.lib.subsystems.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonHelper leftCamera;
    private final PhotonHelper rightCamera;
    private final MeasurementProvider measurementProvider;
    public int lastId = -1;

    private final StructPublisher<Pose2d> predictleftPose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/PredictPose/left", Pose2d.struct).publish();
        private final StructPublisher<Pose2d> predictrightPose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/PredictPose/right", Pose2d.struct).publish();
    
    public VisionSubsystem(MeasurementProvider measurementProvider) {
        super("Vision", false);
        this.leftCamera = new PhotonHelper(
            "left-Cam",
            new Translation3d(0.259679, 0.236378, 0.224837),
            Units.degreesToRadians(-10.0), Units.degreesToRadians(-15.0));
        this.rightCamera = new PhotonHelper(
            "right-Cam",
            new Translation3d(0.259679, -0.236378, 0.224837),
            Units.degreesToRadians(-10.0), Units.degreesToRadians(15.0));
        this.measurementProvider = measurementProvider;
    }

    public Pose2d getFieldToTagPose() {
        int leftCameraId = this.leftCamera.getTagId();
        int rightCameraId = this.rightCamera.getTagId();
        int id;

        if (leftCameraId < 0 && rightCameraId < 0) return null;
        if (leftCameraId != rightCameraId && leftCameraId >= 0 && rightCameraId >= 0) return null;

        id = (rightCameraId < 0) ? leftCameraId : rightCameraId;

        lastId = id;
        return new Pose2d(AprilTagPose.getTranslation3d(id).toTranslation2d(), AprilTagPose.getRotation2d(id));
    }

    public Pose2d getAprilTagFieldPoseFromLastUpdate() {
        Pose2d pose = this.getFieldToTagPose();
        if (pose != null) return pose;
        if (lastId >= 0) return new Pose2d(AprilTagPose.getTranslation3d(lastId).toTranslation2d(), AprilTagPose.getRotation2d(lastId));
        return null;
    }

    public void updateVisionMeasurement() {
        boolean left = this.leftCamera.hasTarget();
        boolean right = this.rightCamera.hasTarget();
        
        if (left && right) {
            Pose2d leftPose = this.leftCamera.getRobotField();
            Pose2d rightPose = this.rightCamera.getRobotField();
            Pose2d finalPose = new Pose2d((leftPose.getX() + rightPose.getX()) / 2.0, (leftPose.getY() + rightPose.getY()) / 2.0, leftPose.getRotation());
            this.measurementProvider.addVisionMeasurement(finalPose, (this.leftCamera.getTimeStamp() + this.rightCamera.getTimeStamp()) / 2.0);
        } else if (left && !right) {
            this.measurementProvider.addVisionMeasurement(this.leftCamera.getRobotField(), this.leftCamera.getTimeStamp());
        } else if (!left && right) {
            this.measurementProvider.addVisionMeasurement(this.rightCamera.getRobotField(), this.rightCamera.getTimeStamp());
        }
    }

    @Override
    public void periodic() {
        this.predictleftPose.accept(this.leftCamera.getRobotField());
        this.predictrightPose.accept(this.rightCamera.getRobotField());
        this.updateVisionMeasurement();
    }

    @Override
    public void putDashboard() {
        // System.out.println("A");
        SmartDashboard.putString("Vision/left/Robot to Tag",this.leftCamera.getRobotToTagPose().toString());
        SmartDashboard.putString("Vision/left/Field to Robot", this.leftCamera.getRobotField().toString());
        SmartDashboard.putString("Vision/right/Robot to Tag",this.rightCamera.getRobotToTagPose().toString());
        SmartDashboard.putString("Vision/right/Field to Robot", this.rightCamera.getRobotField().toString());
    }
}
