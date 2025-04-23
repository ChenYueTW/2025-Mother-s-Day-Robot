package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.helpers.RebuildPhotonHelper;
import frc.robot.lib.helpers.RebuildPhotonHelper.EstimateConsumer;
import frc.robot.lib.subsystems.SubsystemBase;

public class RebuildVisionSubsystem extends SubsystemBase {
    private final RebuildPhotonHelper leftCamera;
    private final RebuildPhotonHelper rightCamera;
    
    public RebuildVisionSubsystem(EstimateConsumer estimateConsumer) {
        super("RebuildVision");
        this.leftCamera = new RebuildPhotonHelper(
            "left-Cam",
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0),
            estimateConsumer);
        this.rightCamera = new RebuildPhotonHelper(
            "right-Cam",
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0),
            estimateConsumer);
    }

    @Override
    public void periodic() {
        this.leftCamera.periodic();
        this.rightCamera.periodic();
    }

    @Override
    public void putDashboard() {
        
    }
}
