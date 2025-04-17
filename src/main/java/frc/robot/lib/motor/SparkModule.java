package frc.robot.lib.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkModule extends SparkMax {
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder encoder;

    public SparkModule(int port, boolean reverse, boolean isBrake) {
        super(port, MotorType.kBrushless);
        this.config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        this.config.inverted(reverse);
        this.configure(this.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.encoder = this.getEncoder();
    }

    // RPM
    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    // Degrees
    public double getPosition() {
        return this.encoder.getPosition();
    }
}
