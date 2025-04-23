package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LevelPosition.ElevatorLevel;
import frc.robot.LevelPosition.LifterLevel;
import frc.robot.commands.CarriageCmd;
import frc.robot.commands.CarriageLifterCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Controller;
import frc.robot.joystick.ControllerButtonStation;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.CarriageLifterSubsystem;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RebuildVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	public final Driver driver = new Driver();
	public final Controller controller = new Controller();
	public final ControllerButtonStation controllerButtonStation = new ControllerButtonStation();
	public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public final CarriageSubsystem carriageSubsystem = new CarriageSubsystem();
	public final CarriageLifterSubsystem carriageLifterSubsystem = new CarriageLifterSubsystem();
	public final FunnelSubsystem funnelSubsystem = new FunnelSubsystem();
	public final LEDSubsystem ledSubsystem = new LEDSubsystem();
	private final RebuildVisionSubsystem rebuildVisionSubsystem = new RebuildVisionSubsystem(this.swerveSubsystem::addVisionMeasurement);
	public final VisionSubsystem visionSubsystem = new VisionSubsystem(this.swerveSubsystem::getGyroAngle);

	// Mode
	public static boolean pushingCoral = false;
	public static boolean funneling = false;
	
	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(new SwerveDriveCmd(
			this.swerveSubsystem, this.visionSubsystem,
			driver::getXDesiredSpeed, driver::getYDesiredSpeed, driver::getRotationSpeed,
			driver::robotMode, driver::trackLeftTag, driver::trackRightTag, driver::getAButton));
		this.elevatorSubsystem.setDefaultCommand(new ElevatorCmd(
			this.elevatorSubsystem, this.controller::lifterElevator));
		this.carriageLifterSubsystem.setDefaultCommand(new CarriageLifterCmd(
			this.carriageLifterSubsystem, this.controller::lifterIntake));
		this.carriageSubsystem.setDefaultCommand(new CarriageCmd(
			this.carriageSubsystem, this.controller::getLeftTriggerAxis, this.controller::getRightTriggerAxis));
		this.ledSubsystem.setDefaultCommand(new LEDCommand(
			this.ledSubsystem, this.carriageSubsystem::getIR));

		this.driver.resetGyro().onTrue(
			Commands.runOnce(this.swerveSubsystem::resetGyro, this.swerveSubsystem));
		this.configButtonStationBindings();
	}

	// Complete
	public void configButtonStationBindings() {
		// Auto Put Coral
		this.controllerButtonStation.coralL1().whileTrue(this.carriageSubsystem.releaseCommand());
		this.configCoralButtonStation(this.controllerButtonStation.coralL2(), ElevatorLevel.L2, LifterLevel.L2);
		this.configCoralButtonStation(this.controllerButtonStation.coralL3(), ElevatorLevel.L3, LifterLevel.L3);
		this.configCoralButtonStation(this.controllerButtonStation.coralL4(), ElevatorLevel.L4, LifterLevel.L4);
		this.configAlgaeButtonStation(this.controllerButtonStation.algaeHigh(), ElevatorLevel.High, LifterLevel.High);
		this.configAlgaeButtonStation(this.controllerButtonStation.algaeLow(), ElevatorLevel.Low, LifterLevel.Low);
		this.controllerButtonStation.pushCoral().onTrue(this.levelToSource());

		// TODO Auto Processor
		this.controllerButtonStation.pushAlgae()
			.onTrue(Commands.sequence(
				this.elevatorSubsystem.autoHoldToPosition(ElevatorLevel.Processer)
					.raceWith(this.elevatorSubsystem.atSetpoint()
						.andThen(this.carriageSubsystem.pushAlgaeToProcessor())),
				this.carriageLifterSubsystem.autoMoveToPosition(LifterLevel.Source)
					.andThen(this.goodFunnel())));

		// Funnel Recive Coral
		this.controllerButtonStation.funnel()
			.whileTrue(this.goodFunnel())
			.onFalse(Commands.parallel(
				Commands.runOnce(this.carriageSubsystem::stopModules, this.carriageSubsystem),
				Commands.runOnce(this.funnelSubsystem::stopModules, this.funnelSubsystem),
				Commands.runOnce(() -> funneling = false)));
		
		// Funnel Release Coral
		this.controllerButtonStation.releaseFunnel()
			.whileTrue(this.funnelSubsystem.executeCommand());

		// Stop Mechanical
		this.controllerButtonStation.stop()
			.onTrue(Commands.runOnce(() -> {
				this.elevatorSubsystem.stopMoveToPositionAndReset();
				this.carriageLifterSubsystem.setVoltage(0.0);
				this.carriageSubsystem.stopRecive();
				this.funnelSubsystem.stopRecive();
				funneling = false;
				pushingCoral = false;
			}, this.elevatorSubsystem, this.carriageLifterSubsystem, this.carriageSubsystem, this.funnelSubsystem, this.ledSubsystem));
	}

	// FINAL
	public void configCoralButtonStation(Trigger button, ElevatorLevel elevatorLevel, LifterLevel lifterLevel) {
		button
			.onTrue(this.levelCoral(elevatorLevel, lifterLevel));
	}


	// FINAL
	public void configAlgaeButtonStation(Trigger button, ElevatorLevel elevatorLevel, LifterLevel lifterLevel) {
		button
			.onTrue(this.autoGetReefAlgae(elevatorLevel, lifterLevel));
	}

	// TELE FIANL
	public Command levelCoral(ElevatorLevel elevatorLevel, LifterLevel lifterLevel) {
		return this.carriageLifterSubsystem.autoHoldToPosition(lifterLevel)
			.alongWith(this.carriageLifterSubsystem.atSetpoint()
				.andThen(this.elevatorSubsystem.autoHoldToPosition(elevatorLevel)));
	}

	// TELE FINAL
	public Command levelToSource() {
		return Commands.sequence(
			this.elevatorSubsystem.holdPosition()
				.alongWith(Commands.runOnce(() -> pushingCoral = true))
				.raceWith(this.elevatorSubsystem.atSetpoint()
					.andThen(this.carriageSubsystem.pushCoralToReefWithIR())),
			this.elevatorSubsystem.autoHoldToPosition(ElevatorLevel.Source)
				.raceWith(this.elevatorSubsystem.atSetpoint()
					.andThen(this.carriageLifterSubsystem.autoMoveToPosition(LifterLevel.Source)
						.andThen(this.goodFunnel()
							.alongWith(Commands.runOnce(() -> pushingCoral = false))))));
	}

	// TELE FINAL
	public Command levelCoralToSource(ElevatorLevel elevatorLevel, LifterLevel lifterLevel) {
		return Commands.sequence(
			this.carriageLifterSubsystem.autoHoldToPosition(lifterLevel)
				.raceWith(this.carriageLifterSubsystem.atSetpoint()
					.andThen(this.elevatorSubsystem.autoHoldToPosition(elevatorLevel)
						.raceWith(this.elevatorSubsystem.atSetpoint()
							.andThen(this.carriageSubsystem.pushCoralToReefWithIR())))),
			this.elevatorSubsystem.autoHoldToPosition(ElevatorLevel.Source)
				.raceWith(this.elevatorSubsystem.atSetpoint()
					.andThen(this.carriageLifterSubsystem.autoMoveToPosition(LifterLevel.Source))),
			this.goodFunnel());
	}

	// TELE FIANL
	public Command autoGetReefAlgae(ElevatorLevel elevatorLevel, LifterLevel lifterLevel) {
		return this.carriageLifterSubsystem.autoMoveToPosition(LifterLevel.L2)
			.andThen(this.elevatorSubsystem.autoHoldToPosition(elevatorLevel)
				.raceWith(this.elevatorSubsystem.atHalfSetpoint()
					.andThen(this.carriageLifterSubsystem.autoHoldToPosition(lifterLevel)
						.raceWith(this.carriageLifterSubsystem.atSetpoint()
							.andThen(this.carriageSubsystem.getAlgaeFromReef())))));
	}

	// TELE FINAL
	public Command autoGetReefAlgaeToProcessor() {
		return this.carriageSubsystem.getAlgaeFromReef()
			.alongWith(this.elevatorSubsystem.autoHoldToPosition(ElevatorLevel.Processer)
				.alongWith(this.carriageLifterSubsystem.autoHoldToPosition(LifterLevel.Processer)));
	}

	// TELE FIANL
	public Command autoPushReefAlgae() {
		return Commands.sequence(
			this.carriageSubsystem.pushAlgaeToProcessor(),
			this.carriageLifterSubsystem.autoMoveToPosition(LifterLevel.L2),
			this.elevatorSubsystem.autoHoldToPosition(ElevatorLevel.Source)
				.raceWith(this.elevatorSubsystem.atSetpoint()
					.andThen(this.carriageLifterSubsystem.autoMoveToPosition(LifterLevel.Source))),
			this.goodFunnel());
	}

	public Command goodFunnel() {
		return this.funnelSubsystem.magicFunnel()
			.alongWith(this.carriageSubsystem.lockCarriage()
				.alongWith(Commands.runOnce(() -> funneling = true)))
			.until(() -> this.carriageSubsystem.getIR())
			.andThen(this.carriageSubsystem.unlockCarriage()
				.withTimeout(1.0)
				.alongWith(Commands.runOnce(() -> funneling = false))
				.andThen(new WaitCommand(0.1)
					.andThen(this.carriageLifterSubsystem.autoMoveToPosition(LifterLevel.L2))));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
