package frc.robot.lib.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.SwerveConstants;

public class SwerveTalon extends TalonFX {
    private double gearRatio;

    @SuppressWarnings("removal")
    public SwerveTalon(int motorPort, boolean reverse, double gearRatio) {
        super(motorPort);
        // this.clearStickyFaults();
        this.setNeutralMode(NeutralModeValue.Brake);
        this.setInverted(reverse);
        // this.resetSignalFrequencies();

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(35.0)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(35.0);
        currentConfig.SupplyCurrentLimit = 120.0;
        currentConfig.SupplyCurrentLimitEnable = true;
        currentConfig.SupplyCurrentLowerLimit = 40.0;

        TalonFXConfiguration FXConfig = new TalonFXConfiguration().withCurrentLimits(currentConfig);
        this.getConfigurator().refresh(FXConfig);
        this.gearRatio = gearRatio;
    }

    public void set(double speed) {
        super.set(speed);
    }

    public double getMotorVelocity() {
        return this.getVelocity().getValueAsDouble() * this.gearRatio * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI;
    }
    
    public double getMotorPosition() {
        return this.getPosition().getValueAsDouble() * this.gearRatio * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI;
    }
}
