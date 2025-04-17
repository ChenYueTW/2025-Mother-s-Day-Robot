package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.CarriageSubsystem;

public class CarriageCmd extends Command {
	private final CarriageSubsystem carriageSubsystem;
	private final Supplier<Double> leftAxis, rightAxis;

	public CarriageCmd(CarriageSubsystem carriageSubsystem, Supplier<Double> leftAxis, Supplier<Double> rightAxis) {
		this.carriageSubsystem = carriageSubsystem;
		this.leftAxis = leftAxis;
		this.rightAxis = rightAxis;
		this.addRequirements(this.carriageSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(
			(this.leftAxis.get() != 0.0) ? -this.leftAxis.get() : this.rightAxis.get(),
			SwerveConstants.DEAD_BAND) * 0.2;
		this.carriageSubsystem.execute(speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.carriageSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
