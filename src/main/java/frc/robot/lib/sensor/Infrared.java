package frc.robot.lib.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class Infrared extends DigitalInput {
    public Infrared(int port) {
        super(port);
    }

    public boolean get() {
        return super.get();
    }
}
