package frc.robot.lib.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

public class TurnEncoder extends CANcoder {
    public TurnEncoder(int port) {
        super(port);
    }

    public double getAbsolutePositionDegrees() {
        double position = Units.rotationsToDegrees(super.getAbsolutePosition().getValueAsDouble());
        position %= 360.0;
        return position > 180 ? position - 360 : position;
    }

    public double getAbsolutePositionRotations() {
        double position = super.getAbsolutePosition().getValueAsDouble();
        position %= 1.0;
        return position > 0.5 ? position - 1.0 : position;
        // return position;
    }
}
