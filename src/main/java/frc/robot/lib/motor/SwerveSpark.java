package frc.robot.lib.motor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.SwerveConstants;

public class SwerveSpark extends SparkMax {
    private final SparkMaxConfig config = new SparkMaxConfig();

    public SwerveSpark(int motorPort, boolean reverse) {
        super(motorPort, MotorType.kBrushless);
        this.config.idleMode(IdleMode.kBrake);
        this.config.inverted(reverse);
        this.config.smartCurrentLimit(SwerveConstants.MAX_VOLTAGE);
        this.configure(this.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
