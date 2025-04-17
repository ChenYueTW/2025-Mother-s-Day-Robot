package frc.robot.lib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class CommandBase extends Command {
    public CommandBase(SubsystemBase subsystemBase) {
        RobotProvider.registerCommands(this);
        this.addRequirements(subsystemBase);
    }

    @Override
    public abstract void initialize();

    @Override
    public abstract void execute();

    @Override
    public abstract void end(boolean interrupted);

    @Override
    public abstract boolean isFinished();
}
