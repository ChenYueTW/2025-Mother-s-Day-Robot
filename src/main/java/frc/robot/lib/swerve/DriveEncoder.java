package frc.robot.lib.swerve;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class DriveEncoder implements RelativeEncoder {
    private final RelativeEncoder encoder;
    private final boolean reverse;

    public DriveEncoder(RelativeEncoder encoder, boolean reverse) {
        this.encoder = encoder;
        this.reverse = reverse;
    }

    @Override
    public double getPosition() {
        return this.encoder.getPosition() * (this.reverse ? 1 : -1);
    }

    @Override
    public double getVelocity() {
        return this.encoder.getVelocity() * (this.reverse ? 1 : -1);
    }

    @Override
    public REVLibError setPosition(double position) {
        return this.encoder.setPosition(position);
    }
}
