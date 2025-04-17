package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCmd extends Command {
	private final ElevatorSubsystem elevatorSubsystem;
	private final Supplier<Double> speed;

	public ElevatorCmd(ElevatorSubsystem elevatorSubsystem, Supplier<Double> speed) {
		this.elevatorSubsystem = elevatorSubsystem;
		this.speed = speed;
		this.addRequirements(this.elevatorSubsystem);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		this.elevatorSubsystem.setVoltage(this.speed.get() * 2.5);
	}

	@Override
	public void end(boolean interrupted) {
		this.elevatorSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
