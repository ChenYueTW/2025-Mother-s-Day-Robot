package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command {
	private final LEDSubsystem ledSubsystem;
	private final Supplier<Boolean> getIR;

	public LEDCommand(LEDSubsystem ledSubsystem, Supplier<Boolean> getIR) {
		this.ledSubsystem = ledSubsystem;
		this.getIR = getIR;
		this.addRequirements(this.ledSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (!RobotState.isAutonomous()) {
			if (RobotContainer.pushingCoral) {
				this.ledSubsystem.setIdleMode(false);
				this.ledSubsystem.pushingCoral();
			} else if (RobotContainer.funneling) {
				this.ledSubsystem.setIdleMode(false);
				this.ledSubsystem.funnel();
			} else if (this.getIR.get()) {
				this.ledSubsystem.setIdleMode(false);
				this.ledSubsystem.hasCoral();
			} else {
				this.ledSubsystem.setIdleMode(true);
			}
		} else {
			this.ledSubsystem.setIdleMode(false);
			this.ledSubsystem.autonomous();
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.ledSubsystem.setIdleMode(true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
