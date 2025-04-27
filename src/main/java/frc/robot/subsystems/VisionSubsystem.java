package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.camera.PhotonHelper;
import frc.robot.lib.camera.PhotonHelper.MeasurementProvider;
import frc.robot.lib.subsystems.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonHelper leftCamera;
    private final PhotonHelper rightCamera;
    private final MeasurementProvider measurementProvider;

    private final StructPublisher<Pose2d> predictleftPose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/PredictPose/left", Pose2d.struct).publish();
        private final StructPublisher<Pose2d> predictrightPose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/PredictPose/right", Pose2d.struct).publish();
    
    public VisionSubsystem(MeasurementProvider measurementProvider) {
        super("Vision", false);
        this.leftCamera = new PhotonHelper(
            "left-Cam",
            new Translation3d(0.261372, 0.236374, 0.224837),
            Units.degreesToRadians(-10.0), Units.degreesToRadians(-15.0));
        this.rightCamera = new PhotonHelper(
            "right-Cam",
            new Translation3d(0.261372, -0.236374, 0.224837),
            Units.degreesToRadians(-10.0), Units.degreesToRadians(15.0));
        this.measurementProvider = measurementProvider;
    }

    public Pose2d getFieldToTagPose() {
        boolean leftHasTarget = this.leftCamera.hasTarget();
        boolean rightHasTarget = this.rightCamera.hasTarget();

        if (leftHasTarget && rightHasTarget) return this.leftCamera.getFieldToTagPose();
        else if (leftHasTarget && !rightHasTarget) return this.leftCamera.getFieldToTagPose();
        else if (!leftHasTarget && rightHasTarget) return this.rightCamera.getFieldToTagPose();
        else return null;
    }

    @Override
    public void periodic() {
        this.predictleftPose.accept(this.leftCamera.getRobotField());
        this.predictrightPose.accept(this.rightCamera.getRobotField());
        if (this.leftCamera.hasTarget())  this.measurementProvider.addVisionMeasurement(this.leftCamera.getRobotField(), this.leftCamera.getTimeStamp());  
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putString("Vision/left/Robot to Tag",this.leftCamera.getRobotToTagPose().toString());
        SmartDashboard.putString("Vision/left/Field to Robot", this.leftCamera.getRobotField().toString());
        SmartDashboard.putString("Vision/right/Robot to Tag",this.rightCamera.getRobotToTagPose().toString());
        SmartDashboard.putString("Vision/right/Field to Robot", this.rightCamera.getRobotField().toString());
    }
}
