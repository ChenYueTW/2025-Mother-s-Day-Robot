package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {
    private final TalonModule funnel = new TalonModule(20, true, false);
    private final double FUNNEL_READY = 0.3;
    private final double FUNNEL_SPEED = 0.0;

    public FunnelSubsystem() {
        super("Funnel", false);
    }

    public void execute() {
        this.funnel.set(this.FUNNEL_SPEED);
    }

    public Command executeCommand() {
        return runEnd(() -> this.funnel.set(0.1), this.funnel::stopMotor);
    }

    public void executeReady() {
        this.funnel.set(-this.FUNNEL_READY);
    }

    public Command magicFunnel() {
        return Commands.repeatingSequence(
            this.funnelReady().withTimeout(0.15),
            runOnce(() -> this.funnel.set(0)),
            new WaitCommand(0.05));
    }

    public void release() {
        this.funnel.set(-this.FUNNEL_SPEED);
    }

    public Command funnelReady() {
        return runEnd(() -> this.funnel.set(-0.3), this::stopModules);
    }

    public void stopModules() {
        this.funnel.stopMotor();
    }

    public void stopRecive() {
        this.funnel.setVoltage(0.0);
    }

    @Override
    public void putDashboard() {}

    @Override
    public void periodic() {}
}
