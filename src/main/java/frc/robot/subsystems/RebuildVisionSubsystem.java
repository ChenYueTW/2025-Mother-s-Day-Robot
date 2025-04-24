package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
            new Translation3d(0.261372, 0.236374, 0.224837),
            new Rotation3d(0.0, Units.degreesToRadians(10.0), Units.degreesToRadians(-15.0)),
            estimateConsumer);
        this.rightCamera = new RebuildPhotonHelper(
            "right-Cam",
            new Translation3d(0.261372, -0.236374, 0.224837),
            new Rotation3d(0.0, Units.degreesToRadians(10.0), Units.degreesToRadians(15.0)),
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
