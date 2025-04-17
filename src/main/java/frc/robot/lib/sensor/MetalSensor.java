package frc.robot.lib.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class MetalSensor extends DigitalInput {
    public MetalSensor(int port) {
        super(port);
    }

    public boolean get() {
        return !super.get();
    }
}
